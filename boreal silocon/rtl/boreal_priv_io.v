// ============================================================================
// Boreal SoC - Privileged I/O
// ============================================================================
// Register bank only accessible by the Gate master port.  Represents the
// physical actuator / output registers that AI actions ultimately drive.
// 256 x 32-bit registers (1 KB).
// ============================================================================

module boreal_priv_io (
    input  wire        clk,
    input  wire        rst_n,

    // --- Slave interface (Gate only) ---
    input  wire        sel,
    input  wire        wr,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output reg         ack,

    // --- Physical output pins (directly driven by registers) ---
    output wire [31:0] pio_out_0,
    output wire [31:0] pio_out_1,
    output wire [31:0] pio_out_2,
    output wire [31:0] pio_out_3
);

    // 256 x 32-bit register file
    reg [31:0] regs [0:255];

    wire [7:0] reg_idx = addr[9:2];

    integer i;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 256; i = i + 1)
                regs[i] <= 32'h0;
        end else if (sel && wr) begin
            regs[reg_idx] <= wdata;
        end
    end

    always @(*) begin
        rdata = 32'h0;
        ack   = sel;
        if (sel && !wr)
            rdata = regs[reg_idx];
    end

    // Map first 4 registers to physical output pins
    assign pio_out_0 = regs[0];
    assign pio_out_1 = regs[1];
    assign pio_out_2 = regs[2];
    assign pio_out_3 = regs[3];

endmodule
