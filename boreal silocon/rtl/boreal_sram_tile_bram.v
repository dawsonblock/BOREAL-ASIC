// ============================================================================
// Boreal SoC - SRAM Tile (BRAM-backed)
// ============================================================================
// 4 KB (1024 x 32-bit) synchronous SRAM tile inferred as block RAM.
// Supports byte-lane strobes for sub-word writes.
// ============================================================================

module boreal_sram_tile_bram #(
    parameter DEPTH     = 1024,
    parameter DEPTH_LOG = 10
)(
    input  wire        clk,
    input  wire        rst_n,

    // --- Bus slave interface ---
    input  wire        sel,
    input  wire        wr,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    input  wire [ 3:0] strb,
    output reg  [31:0] rdata,
    output reg         ack,

    // --- DMA port (secondary, lower priority) ---
    input  wire        dma_sel,
    input  wire        dma_wr,
    input  wire [DEPTH_LOG-1:0] dma_addr,
    input  wire [31:0] dma_wdata,
    output reg  [31:0] dma_rdata,
    output reg         dma_ack
);

    // BRAM storage
    reg [31:0] mem [0:DEPTH-1];

    wire [DEPTH_LOG-1:0] bus_word_addr = addr[DEPTH_LOG+1:2];

    // Byte-lane write helper
    wire [31:0] strb_mask = {{8{strb[3]}}, {8{strb[2]}}, {8{strb[1]}}, {8{strb[0]}}};

    // Bus port has priority
    wire bus_active = sel;
    wire dma_active = dma_sel && !bus_active;

    // -----------------------------------------------------------------------
    // Write
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (bus_active && wr) begin
            mem[bus_word_addr] <= (mem[bus_word_addr] & ~strb_mask) | (wdata & strb_mask);
        end else if (dma_active && dma_wr) begin
            mem[dma_addr] <= dma_wdata;
        end
    end

    // -----------------------------------------------------------------------
    // Read – registered output (1-cycle latency)
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (bus_active && !wr)
            rdata <= mem[bus_word_addr];
        else
            rdata <= 32'h0;

        if (dma_active && !dma_wr)
            dma_rdata <= mem[dma_addr];
        else
            dma_rdata <= 32'h0;
    end

    // -----------------------------------------------------------------------
    // Ack generation (1-cycle)
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ack     <= 1'b0;
            dma_ack <= 1'b0;
        end else begin
            ack     <= bus_active;
            dma_ack <= dma_active;
        end
    end

endmodule
