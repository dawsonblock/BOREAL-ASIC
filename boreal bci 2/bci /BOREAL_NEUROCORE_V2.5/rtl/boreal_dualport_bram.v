/**
 * Boreal Dual-Port BRAM with CDC
 */

module boreal_dualport_bram #(
    parameter ADDR_WIDTH = 10,
    parameter DATA_WIDTH = 32,
    parameter INIT_FILE = "boreal_lut.mem"
)(
    input  wire                  clk_a,
    input  wire                  rst_n_a,
    input  wire [ADDR_WIDTH-1:0] addr_a,
    output reg  [DATA_WIDTH-1:0] dout_a,
    input  wire                  clk_b,
    input  wire                  rst_n_b,
    input  wire                  we_b,
    input  wire [ADDR_WIDTH-1:0] addr_b,
    input  wire [DATA_WIDTH-1:0] din_b,
    output reg  [DATA_WIDTH-1:0] dout_b,
    output wire                  collision_detected
);

    reg [DATA_WIDTH-1:0] ram [0:(2**ADDR_WIDTH)-1];
    reg [ADDR_WIDTH-1:0] addr_a_reg, addr_b_reg;

    always @(posedge clk_a) addr_a_reg <= addr_a;
    always @(posedge clk_b) addr_b_reg <= addr_b;
    assign collision_detected = (addr_a_reg == addr_b_reg) && we_b;

    initial begin
        if (INIT_FILE != "")
            $readmemh(INIT_FILE, ram);
    end

    always @(posedge clk_a) begin
        if (!rst_n_a)
            dout_a <= {DATA_WIDTH{1'b0}};
        else
            dout_a <= ram[addr_a];
    end

    always @(posedge clk_b) begin
        if (!rst_n_b)
            dout_b <= {DATA_WIDTH{1'b0}};
        else begin
            if (we_b)
                ram[addr_b] <= din_b;
            dout_b <= ram[addr_b];
        end
    end
endmodule
