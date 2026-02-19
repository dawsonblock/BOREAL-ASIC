`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_vector_engine.v  (local scratchpad + 8-lane int8 MAC)
// - CPU loads A and B vectors into scratchpad via MMIO window.
// - Start triggers compute: acc[i] += A[i]*B[i] for LEN elements (LEN mult of 8)
// - Output stored in OUT scratchpad (32-bit per lane per step).
// - Deterministic runtime: LEN/8 cycles.
// ============================================================================
`include "boreal_pkg.vh"

module boreal_vector_engine #(
    parameter SP_WORDS = 2048
) (
    input  wire        clk,
    input  wire        rst_n,

    // MMIO (word offsets within 0x1001_0000)
    input  wire        mmio_we,
    input  wire [10:0] mmio_addr,
    input  wire [31:0] mmio_wdata,
    input  wire [3:0]  mmio_wstrb,
    output reg  [31:0] mmio_rdata
);
    // control regs
    reg [31:0] VEC_CMD;
    reg [31:0] VEC_LEN;
    reg [31:0] VEC_STATUS;

    // scratchpad: A @ 0x100, B @ 0x300, OUT @ 0x500
    reg [31:0] sp [0:SP_WORDS-1];

    // engine
    reg        busy;
    reg [31:0] steps;
    reg [31:0] step_i;

    // compute temporaries (module-scope for iverilog)
    reg [31:0] a0_w, a1_w, b0_w, b1_w;
    reg signed [31:0] acc0, acc1, acc2, acc3, acc4, acc5, acc6, acc7;

    // byte extraction helper
    function signed [7:0] byte_at;
        input [31:0] w;
        input integer b;
        begin
            case (b)
                0: byte_at = w[ 7: 0];
                1: byte_at = w[15: 8];
                2: byte_at = w[23:16];
                3: byte_at = w[31:24];
                default: byte_at = 8'd0;
            endcase
        end
    endfunction

    integer i;
    initial begin
        for (i = 0; i < SP_WORDS; i = i + 1) sp[i] = 32'h0;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            VEC_CMD    <= 32'h0;
            VEC_LEN    <= 32'h0;
            VEC_STATUS <= 32'h0;
            mmio_rdata <= 32'h0;
            busy   <= 1'b0;
            step_i <= 32'h0;
            steps  <= 32'h0;
        end else begin
            // MMIO writes
            if (mmio_we) begin
                if (mmio_addr == 10'h000)      VEC_CMD <= mmio_wdata;
                else if (mmio_addr == 10'h001) VEC_LEN <= mmio_wdata;
                else if (mmio_addr < SP_WORDS)
                    sp[mmio_addr] <= mmio_wdata;
            end

            // MMIO reads
            if (mmio_addr == 10'h000)           mmio_rdata <= VEC_CMD;
            else if (mmio_addr == 10'h001)      mmio_rdata <= VEC_LEN;
            else if (mmio_addr == 10'h009)      mmio_rdata <= VEC_STATUS;
            else if (mmio_addr < SP_WORDS) mmio_rdata <= sp[mmio_addr];
            else                                mmio_rdata <= 32'h0;

            // start condition
            if (!busy && VEC_CMD[0]) begin
                busy       <= 1'b1;
                VEC_STATUS <= 32'h1; // busy
                VEC_CMD[0] <= 1'b0;  // self-clear
                steps      <= (VEC_LEN >> 3);
                step_i     <= 32'h0;
            end

            // compute (1 step per cycle, 8 elements per step)
            if (busy) begin
                if (step_i >= steps) begin
                    busy       <= 1'b0;
                    VEC_STATUS <= 32'h2; // done
                end else begin
                    a0_w = sp[11'h100 + (step_i << 1)];
                    a1_w = sp[11'h100 + (step_i << 1) + 1];
                    b0_w = sp[11'h300 + (step_i << 1)];
                    b1_w = sp[11'h300 + (step_i << 1) + 1];

                    acc0 = $signed(byte_at(a0_w, 0)) * $signed(byte_at(b0_w, 0));
                    acc1 = $signed(byte_at(a0_w, 1)) * $signed(byte_at(b0_w, 1));
                    acc2 = $signed(byte_at(a0_w, 2)) * $signed(byte_at(b0_w, 2));
                    acc3 = $signed(byte_at(a0_w, 3)) * $signed(byte_at(b0_w, 3));
                    acc4 = $signed(byte_at(a1_w, 0)) * $signed(byte_at(b1_w, 0));
                    acc5 = $signed(byte_at(a1_w, 1)) * $signed(byte_at(b1_w, 1));
                    acc6 = $signed(byte_at(a1_w, 2)) * $signed(byte_at(b1_w, 2));
                    acc7 = $signed(byte_at(a1_w, 3)) * $signed(byte_at(b1_w, 3));

                    sp[11'h500 + (step_i << 3) + 0] <= acc0;
                    sp[11'h500 + (step_i << 3) + 1] <= acc1;
                    sp[11'h500 + (step_i << 3) + 2] <= acc2;
                    sp[11'h500 + (step_i << 3) + 3] <= acc3;
                    sp[11'h500 + (step_i << 3) + 4] <= acc4;
                    sp[11'h500 + (step_i << 3) + 5] <= acc5;
                    sp[11'h500 + (step_i << 3) + 6] <= acc6;
                    sp[11'h500 + (step_i << 3) + 7] <= acc7;

                    step_i <= step_i + 1;
                end
            end
        end
    end
endmodule
`default_nettype wire
