// ============================================================================
// Boreal SoC - Append-Only Ledger
// ============================================================================
// Circular buffer of 1024 x 256-bit entries stored in BRAM.  Write-once
// per index (entries are never modified after commit).  Provides MMIO
// read interface for audit / replay.
// ============================================================================

module boreal_ledger #(
    parameter DEPTH     = 1024,
    parameter DEPTH_LOG = 10
)(
    input  wire        clk,
    input  wire        rst_n,

    // --- Write port (from Gate) ---
    input  wire        wr_en,
    input  wire [255:0] wr_data,

    // --- Current index output ---
    output reg  [31:0] idx,

    // --- MMIO slave interface ---
    input  wire        sel,
    input  wire        wr,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output reg         ack
);

    // Register offsets
    localparam OFF_IDX      = 8'h00;
    localparam OFF_DEPTH    = 8'h04;
    localparam OFF_RD_ADDR  = 8'h08;
    localparam OFF_RD_DATA0 = 8'h0C;
    localparam OFF_RD_DATA1 = 8'h10;
    localparam OFF_RD_DATA2 = 8'h14;
    localparam OFF_RD_DATA3 = 8'h18;

    wire [7:0] reg_off = addr[7:0];

    // Entry storage – 8 x 32-bit words per entry for 256 bits
    reg [31:0] mem0 [0:DEPTH-1];
    reg [31:0] mem1 [0:DEPTH-1];
    reg [31:0] mem2 [0:DEPTH-1];
    reg [31:0] mem3 [0:DEPTH-1];
    reg [31:0] mem4 [0:DEPTH-1];
    reg [31:0] mem5 [0:DEPTH-1];
    reg [31:0] mem6 [0:DEPTH-1];
    reg [31:0] mem7 [0:DEPTH-1];

    // Read address register
    reg [DEPTH_LOG-1:0] rd_addr;

    // Read data pipeline
    reg [31:0] rd_data0, rd_data1, rd_data2, rd_data3;
    reg [31:0] rd_data4, rd_data5, rd_data6, rd_data7;

    // -----------------------------------------------------------------------
    // Write logic – append only
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            idx <= 32'd0;
        end else if (wr_en) begin
            mem0[idx[DEPTH_LOG-1:0]] <= wr_data[ 31:  0];
            mem1[idx[DEPTH_LOG-1:0]] <= wr_data[ 63: 32];
            mem2[idx[DEPTH_LOG-1:0]] <= wr_data[ 95: 64];
            mem3[idx[DEPTH_LOG-1:0]] <= wr_data[127: 96];
            mem4[idx[DEPTH_LOG-1:0]] <= wr_data[159:128];
            mem5[idx[DEPTH_LOG-1:0]] <= wr_data[191:160];
            mem6[idx[DEPTH_LOG-1:0]] <= wr_data[223:192];
            mem7[idx[DEPTH_LOG-1:0]] <= wr_data[255:224];
            idx <= idx + 1;
        end
    end

    // -----------------------------------------------------------------------
    // Read pipeline – registered output
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        rd_data0 <= mem0[rd_addr];
        rd_data1 <= mem1[rd_addr];
        rd_data2 <= mem2[rd_addr];
        rd_data3 <= mem3[rd_addr];
        rd_data4 <= mem4[rd_addr];
        rd_data5 <= mem5[rd_addr];
        rd_data6 <= mem6[rd_addr];
        rd_data7 <= mem7[rd_addr];
    end

    // -----------------------------------------------------------------------
    // MMIO interface
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            rd_addr <= {DEPTH_LOG{1'b0}};
        else if (sel && wr && reg_off == OFF_RD_ADDR)
            rd_addr <= wdata[DEPTH_LOG-1:0];
    end

    always @(*) begin
        rdata = 32'h0;
        ack   = sel;
        if (sel && !wr) begin
            case (reg_off)
                OFF_IDX:      rdata = idx;
                OFF_DEPTH:    rdata = DEPTH;
                OFF_RD_DATA0: rdata = rd_data0;
                OFF_RD_DATA1: rdata = rd_data1;
                OFF_RD_DATA2: rdata = rd_data2;
                OFF_RD_DATA3: rdata = rd_data3;
                default:      rdata = 32'h0;
            endcase
        end
    end

endmodule
