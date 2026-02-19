`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_decision_vm.v  (bounded minimal VM)
// - Reads AI mailbox words 0..3 (mb_ai[127:0])
// - Simple rule: clamp r0 to <=100, then EMIT opcode=1 target=0x10 arg0=r1
// - Waits for response valid then done
// - Fixed cycle budget 256 cycles; on budget exceed -> done + reset pc
// ============================================================================
module boreal_decision_vm (
    input  wire         clk,
    input  wire         rst_n,

    // AI mailbox (latched externally into mb_ai words)
    input  wire [127:0] mb_ai,

    // Gate response mailbox
    input  wire         gate_resp_valid,
    input  wire [159:0] gate_resp_data,

    // Gate action request
    output reg          act_valid,
    output reg  [511:0] act_data,

    // Control
    input  wire         start,
    output reg          done
);
    reg [7:0]  pc;
    reg [7:0]  cycle_ctr;
    reg [31:0] r0, r1;

    // nonce local (VM-side) for deterministic emission
    reg [31:0] nonce;

    // policy hash expected (must match gate.policy_hash)
    reg [31:0] policy_hash;

    // bounds: clamp_class=0
    wire [31:0] bounds = 32'h0;

    always @(posedge clk) begin
        if (!rst_n) begin
            pc          <= 8'd0;
            cycle_ctr   <= 8'd0;
            act_valid   <= 1'b0;
            act_data    <= 512'h0;
            done        <= 1'b0;
            r0          <= 32'h0;
            r1          <= 32'h0;
            nonce       <= 32'h0;
            policy_hash <= 32'hA5A5_0001; // Phase-A constant; set gate to same
        end else begin
            act_valid <= 1'b0;

            if (!start) begin
                done      <= 1'b0;
                pc        <= 8'd0;
                cycle_ctr <= 8'd0;
            end else begin
                cycle_ctr <= cycle_ctr + 1;

                if (cycle_ctr >= 8'd255) begin
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
                        8'd2: begin
                            // EMIT: opcode=1, target=0x10, arg0=r1, arg1=0
                            act_data <= {
                                256'd0,
                                nonce,
                                bounds,
                                policy_hash,
                                32'h0,
                                32'h0,
                                r1,
                                32'h0000_0010,
                                32'd1
                            };
                            act_valid <= 1'b1;
                            pc <= 8'd3;
                        end
                        8'd3: begin
                            // Wait for response
                            if (gate_resp_valid) begin
                                done  <= 1'b1;
                                nonce <= nonce + 1;
                                pc    <= 8'd0;
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
