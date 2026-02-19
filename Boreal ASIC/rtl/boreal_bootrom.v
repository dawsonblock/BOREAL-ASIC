`timescale 1ns / 1ps
// ============================================================================
// Boreal SoC - Boot ROM
// ============================================================================
// 4 KB read-only boot ROM.  Contents are loaded from a hex file at
// synthesis time (or initialised to a default boot stub).  Implements
// measured boot: computes SHA-256 hash of the loaded image and checks
// the signature before releasing the system reset.
// ============================================================================

module boreal_bootrom #(
    parameter DEPTH     = 1024,
    parameter DEPTH_LOG = 10,
    parameter INIT_FILE = ""
)(
    input  wire        clk,
    input  wire        rst_n,

    // --- Bus slave interface (read-only) ---
    input  wire        sel,
    input  wire [31:0] addr,
    output reg  [31:0] rdata,
    output reg         ack,

    // --- Boot control ---
    output reg         boot_done,
    output reg         boot_pass,
    output reg  [31:0] boot_hash_out,

    // --- SHA-256 interface ---
    output reg         sha_start,
    output reg         sha_update,
    output reg  [31:0] sha_data,
    input  wire [31:0] sha_hash,
    input  wire        sha_ready,

    // --- Signature verify interface ---
    output reg         sig_start,
    output reg  [31:0] sig_hash_in,
    input  wire        sig_pass,
    input  wire        sig_ready
);

    // ROM storage
    reg [31:0] rom [0:DEPTH-1];

    // Zero-initialise ROM then optionally load from file
    integer ri;
    initial begin
        for (ri = 0; ri < DEPTH; ri = ri + 1)
            rom[ri] = 32'h0;
        if (INIT_FILE != "")
            $readmemh(INIT_FILE, rom);
    end

    wire [DEPTH_LOG-1:0] word_addr = addr[DEPTH_LOG+1:2];

    // -----------------------------------------------------------------------
    // Read interface
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (sel)
            rdata <= rom[word_addr];
        else
            rdata <= 32'h0;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ack <= 1'b0;
        else
            ack <= sel;
    end

    // -----------------------------------------------------------------------
    // Boot sequence FSM
    // -----------------------------------------------------------------------
    localparam BS_IDLE       = 4'd0;
    localparam BS_HASH_INIT  = 4'd1;
    localparam BS_HASH_WAIT  = 4'd2;
    localparam BS_HASH_FEED  = 4'd3;
    localparam BS_HASH_DONE  = 4'd4;
    localparam BS_SIG_START  = 4'd5;
    localparam BS_SIG_WAIT   = 4'd6;
    localparam BS_SIG_CHECK  = 4'd7;
    localparam BS_DONE       = 4'd8;

    reg [3:0]  bs_state;
    reg [31:0] hash_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bs_state     <= BS_IDLE;
            boot_done    <= 1'b0;
            boot_pass    <= 1'b0;
            boot_hash_out<= 32'h0;
            sha_start    <= 1'b0;
            sha_update   <= 1'b0;
            sha_data     <= 32'h0;
            sig_start    <= 1'b0;
            sig_hash_in  <= 32'h0;
            hash_idx     <= {DEPTH_LOG{1'b0}};
        end else begin
            case (bs_state)
                BS_IDLE: begin
                    // Start boot sequence immediately after reset
                    sha_start <= 1'b1;
                    bs_state  <= BS_HASH_INIT;
                end

                // Wait 1 cycle for SHA to deassert ready
                BS_HASH_INIT: begin
                    sha_start <= 1'b0;
                    bs_state  <= BS_HASH_WAIT;
                end

                // Wait for SHA ready after init
                BS_HASH_WAIT: begin
                    if (sha_ready) begin
                        hash_idx <= 32'd0;
                        bs_state <= BS_HASH_FEED;
                    end
                end

                BS_HASH_FEED: begin
                    if (hash_idx >= DEPTH) begin
                        sha_update <= 1'b0;
                        bs_state   <= BS_HASH_DONE;
                    end else if (sha_ready) begin
                        sha_update <= 1'b1;
                        sha_data   <= rom[hash_idx[DEPTH_LOG-1:0]];
                        hash_idx   <= hash_idx + 1;
                    end
                end

                BS_HASH_DONE: begin
                    if (sha_ready) begin
                        boot_hash_out <= sha_hash;
                        sig_hash_in   <= sha_hash;
                        sig_start     <= 1'b1;
                        bs_state      <= BS_SIG_START;
                    end
                end

                // Wait 1 cycle for sigverify to deassert ready
                BS_SIG_START: begin
                    sig_start <= 1'b0;
                    bs_state  <= BS_SIG_WAIT;
                end

                // Wait for sigverify to deassert ready (processing)
                BS_SIG_WAIT: begin
                    if (!sig_ready)
                        bs_state <= BS_SIG_CHECK;
                end

                // Wait for sigverify to reassert ready (done)
                BS_SIG_CHECK: begin
                    if (sig_ready) begin
                        boot_pass <= sig_pass;
                        boot_done <= 1'b1;
                        bs_state  <= BS_DONE;
                    end
                end

                BS_DONE: begin
                    // Stay here – boot complete
                end

                default: bs_state <= BS_IDLE;
            endcase
        end
    end

endmodule
