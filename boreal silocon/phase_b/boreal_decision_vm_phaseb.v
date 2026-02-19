`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_decision_vm_phaseb.v (Phase-B VM using mailbox regs)
// - Reads mb_ai[127:0]
// - Writes req words 0..7 then sets req_valid
// - Polls resp_valid, acks it
// ============================================================================
`include "boreal_pkg.vh"

module boreal_decision_vm_phaseb (
    input  wire         clk,
    input  wire         rst_n,

    input  wire [127:0] mb_ai,

    // mailbox interface
    output reg          vm_req_we,
    output reg  [3:0]   vm_req_widx,
    output reg  [31:0]  vm_req_wdata,
    output reg          vm_req_valid_set,

    input  wire         vm_resp_valid,
    input  wire [31:0]  vm_resp_w0,
    input  wire [31:0]  vm_resp_w1,
    input  wire [31:0]  vm_resp_w2,
    input  wire [31:0]  vm_resp_w3,
    input  wire [31:0]  vm_resp_w4,
    output reg          vm_resp_ack,

    input  wire         start,
    output reg          done
);
    reg [7:0]  pc;
    reg [7:0]  cycle_ctr;
    reg [31:0] r0, r1;
    reg [31:0] nonce;
    reg [31:0] policy_hash;

    always @(posedge clk) begin
        if (!rst_n) begin
            pc          <= 8'd0;
            cycle_ctr   <= 8'd0;
            r0          <= 32'h0;
            r1          <= 32'h0;
            nonce       <= 32'h0;
            policy_hash <= 32'hA5A5_0001;
            vm_req_we        <= 1'b0;
            vm_req_widx      <= 4'h0;
            vm_req_wdata     <= 32'h0;
            vm_req_valid_set <= 1'b0;
            vm_resp_ack      <= 1'b0;
            done             <= 1'b0;
        end else begin
            vm_req_we        <= 1'b0;
            vm_req_valid_set <= 1'b0;
            vm_resp_ack      <= 1'b0;

            if (!start) begin
                done      <= 1'b0;
                pc        <= 8'd0;
                cycle_ctr <= 8'd0;
            end else begin
                cycle_ctr <= cycle_ctr + 1;
                if (cycle_ctr == 8'd255) begin
                    done      <= 1'b1;
                    pc        <= 8'd0;
                    cycle_ctr <= 8'd0;
                end else begin
                    case (pc)
                        8'd0: begin
                            done <= 1'b0;
                            r0   <= mb_ai[31:0];
                            pc   <= 8'd1;
                        end
                        8'd1: begin
                            r1 <= (r0 > 32'd100) ? 32'd100 : r0;
                            pc <= 8'd2;
                        end
                        // write req words 0..7
                        8'd2:  begin vm_req_we<=1; vm_req_widx<=4'd0; vm_req_wdata<=32'd1;         pc<=8'd3;  end
                        8'd3:  begin vm_req_we<=1; vm_req_widx<=4'd1; vm_req_wdata<=32'h0000_0010; pc<=8'd4;  end
                        8'd4:  begin vm_req_we<=1; vm_req_widx<=4'd2; vm_req_wdata<=r1;            pc<=8'd5;  end
                        8'd5:  begin vm_req_we<=1; vm_req_widx<=4'd3; vm_req_wdata<=32'd0;         pc<=8'd6;  end
                        8'd6:  begin vm_req_we<=1; vm_req_widx<=4'd4; vm_req_wdata<=32'd0;         pc<=8'd7;  end
                        8'd7:  begin vm_req_we<=1; vm_req_widx<=4'd5; vm_req_wdata<=policy_hash;   pc<=8'd8;  end
                        8'd8:  begin vm_req_we<=1; vm_req_widx<=4'd6; vm_req_wdata<=32'd0;         pc<=8'd9;  end
                        8'd9:  begin vm_req_we<=1; vm_req_widx<=4'd7; vm_req_wdata<=nonce;         pc<=8'd10; end
                        8'd10: begin
                            vm_req_valid_set <= 1'b1;
                            pc <= 8'd11;
                        end
                        8'd11: begin
                            if (vm_resp_valid) begin
                                vm_resp_ack <= 1'b1;
                                nonce       <= nonce + 1;
                                done        <= 1'b1;
                                pc          <= 8'd0;
                            end
                        end
                        default: pc <= 8'd0;
                    endcase
                end
            end
        end
    end
endmodule
`default_nettype wire
