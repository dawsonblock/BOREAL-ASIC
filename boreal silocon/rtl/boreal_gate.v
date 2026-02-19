// ============================================================================
// Boreal SoC - Central Gate
// ============================================================================
// Non-bypassable action commit engine.  Receives 64-byte action requests
// from the Decision-VM, enforces policy (allowlist, rate-limit, bounds),
// writes sanitised results to Privileged I/O, and commits a ledger entry.
// ============================================================================
`timescale 1ns / 1ps
`include "boreal_pkg.v"

module boreal_gate (
    input  wire        clk,
    input  wire        rst_n,

    // --- Action request input (from Decision-VM) ---
    input  wire        act_valid,
    input  wire [31:0] act_opcode,
    input  wire [31:0] act_target,
    input  wire [31:0] act_arg0,
    input  wire [31:0] act_arg1,
    input  wire [31:0] act_context_hash,
    input  wire [31:0] act_policy_hash,
    input  wire [31:0] act_bounds,
    input  wire [31:0] act_nonce,
    output reg         act_ready,

    // --- Action response output ---
    output reg         resp_valid,
    output reg  [31:0] resp_committed,
    output reg  [31:0] resp_reason,
    output reg  [31:0] resp_applied0,
    output reg  [31:0] resp_applied1,
    output reg  [31:0] resp_ledger_idx,

    // --- Policy configuration (from boreal_gate_policy) ---
    input  wire [31:0] allow0,
    input  wire [31:0] allow1,
    input  wire [31:0] rate_limit,
    input  wire [31:0] rate_window,
    input  wire [31:0] policy_hash,
    input  wire [31:0] override_reg,

    // --- Nonce counter output ---
    output reg  [31:0] nonce_counter,

    // --- Privileged I/O master port ---
    output reg         priv_req,
    output reg         priv_wr,
    output reg  [31:0] priv_addr,
    output reg  [31:0] priv_wdata,
    input  wire        priv_ack,

    // --- Ledger write port ---
    output reg         ledger_wr_en,
    output reg [255:0] ledger_wr_data,
    input  wire [31:0] ledger_idx
);

    // -----------------------------------------------------------------------
    // Internal state
    // -----------------------------------------------------------------------
    localparam S_IDLE      = 3'd0;
    localparam S_CHECK     = 3'd1;
    localparam S_CLAMP     = 3'd2;
    localparam S_COMMIT    = 3'd3;
    localparam S_LEDGER    = 3'd4;
    localparam S_RESPOND   = 3'd5;

    reg [2:0]  state;

    // Latched request fields
    reg [31:0] r_opcode, r_target, r_arg0, r_arg1;
    reg [31:0] r_context_hash, r_policy_hash, r_bounds, r_nonce;

    // Decision results
    reg        committed;
    reg [31:0] reason;
    reg [31:0] applied0, applied1;

    // Rate limiter
    reg [31:0] rate_count;
    reg [31:0] rate_timer;

    // -----------------------------------------------------------------------
    // Rate-limit timer
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rate_count <= 32'd0;
            rate_timer <= 32'd0;
        end else begin
            if (rate_timer >= rate_window) begin
                rate_timer <= 32'd0;
                rate_count <= 32'd0;
            end else begin
                rate_timer <= rate_timer + 1;
            end
            // Increment on successful commit
            if (state == S_COMMIT && committed)
                rate_count <= rate_count + 1;
        end
    end

    // -----------------------------------------------------------------------
    // Nonce – monotonic counter
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            nonce_counter <= 32'd0;
        else if (state == S_LEDGER)
            nonce_counter <= nonce_counter + 1;
    end

    // -----------------------------------------------------------------------
    // Main FSM
    // -----------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            act_ready    <= 1'b1;
            resp_valid   <= 1'b0;
            priv_req     <= 1'b0;
            priv_wr      <= 1'b0;
            ledger_wr_en <= 1'b0;
            committed    <= 1'b0;
            reason       <= 32'd0;
            applied0     <= 32'd0;
            applied1     <= 32'd0;
            r_opcode     <= 32'd0;
            r_target     <= 32'd0;
            r_arg0       <= 32'd0;
            r_arg1       <= 32'd0;
            r_context_hash <= 32'd0;
            r_policy_hash  <= 32'd0;
            r_bounds     <= 32'd0;
            r_nonce      <= 32'd0;
            resp_committed <= 32'd0;
            resp_reason    <= 32'd0;
            resp_applied0  <= 32'd0;
            resp_applied1  <= 32'd0;
            resp_ledger_idx<= 32'd0;
            priv_addr      <= 32'd0;
            priv_wdata     <= 32'd0;
            ledger_wr_data <= 256'd0;
        end else begin
            case (state)
                // ---------------------------------------------------------
                S_IDLE: begin
                    resp_valid   <= 1'b0;
                    ledger_wr_en <= 1'b0;
                    priv_req     <= 1'b0;
                    if (act_valid && act_ready) begin
                        // Latch request
                        r_opcode       <= act_opcode;
                        r_target       <= act_target;
                        r_arg0         <= act_arg0;
                        r_arg1         <= act_arg1;
                        r_context_hash <= act_context_hash;
                        r_policy_hash  <= act_policy_hash;
                        r_bounds       <= act_bounds;
                        r_nonce        <= act_nonce;
                        act_ready      <= 1'b0;
                        state          <= S_CHECK;
                    end
                end

                // ---------------------------------------------------------
                // Policy check
                S_CHECK: begin
                    committed <= 1'b0;
                    reason    <= `REASON_OK;
                    applied0  <= r_arg0;
                    applied1  <= r_arg1;

                    // NOP always succeeds
                    if (r_opcode == `ACT_NOP) begin
                        committed <= 1'b1;
                        state     <= S_LEDGER;
                    end
                    // Nonce must match
                    else if (r_nonce != nonce_counter) begin
                        reason <= `REASON_NONCE_ERROR;
                        state  <= S_LEDGER;
                    end
                    // Policy hash must match (unless overridden)
                    else if (r_policy_hash != policy_hash && override_reg == 32'h0) begin
                        reason <= `REASON_POLICY_DENIED;
                        state  <= S_LEDGER;
                    end
                    // Allowlist check
                    else if (r_target < 32) begin
                        if (allow0[r_target[4:0]]) begin
                            committed <= 1'b1;
                            state     <= S_CLAMP;
                        end else begin
                            reason <= `REASON_POLICY_DENIED;
                            state  <= S_LEDGER;
                        end
                    end else if (r_target < 64) begin
                        if (allow1[r_target[4:0]]) begin
                            committed <= 1'b1;
                            state     <= S_CLAMP;
                        end else begin
                            reason <= `REASON_POLICY_DENIED;
                            state  <= S_LEDGER;
                        end
                    end else begin
                        reason <= `REASON_INVALID;
                        state  <= S_LEDGER;
                    end

                    // Rate limit override
                    if (rate_count >= rate_limit && r_opcode != `ACT_NOP) begin
                        committed <= 1'b0;
                        reason    <= `REASON_RATE_LIMITED;
                        state     <= S_LEDGER;
                    end
                end

                // ---------------------------------------------------------
                // Bounds clamping (simple min/max on arg0)
                S_CLAMP: begin
                    if (r_bounds != 32'h0) begin
                        // Upper 16 bits = max, lower 16 bits = min
                        if (r_arg0 > {16'h0, r_bounds[31:16]}) begin
                            applied0 <= {16'h0, r_bounds[31:16]};
                            reason   <= `REASON_CLAMPED;
                        end else if (r_arg0 < {16'h0, r_bounds[15:0]}) begin
                            applied0 <= {16'h0, r_bounds[15:0]};
                            reason   <= `REASON_CLAMPED;
                        end
                    end
                    state <= S_COMMIT;
                end

                // ---------------------------------------------------------
                // Write sanitised value to Privileged I/O
                S_COMMIT: begin
                    if (committed && r_opcode == `ACT_WRITE) begin
                        priv_req   <= 1'b1;
                        priv_wr    <= 1'b1;
                        priv_addr  <= `ADDR_PRIV_IO_BASE + {r_target[15:0], 2'b00};
                        priv_wdata <= applied0;
                        if (priv_ack) begin
                            priv_req <= 1'b0;
                            state    <= S_LEDGER;
                        end
                    end else begin
                        state <= S_LEDGER;
                    end
                end

                // ---------------------------------------------------------
                // Write ledger entry
                S_LEDGER: begin
                    ledger_wr_en  <= 1'b1;
                    ledger_wr_data <= {
                        rate_timer,          // [255:224] cycle
                        r_nonce,             // [223:192] nonce
                        r_opcode,            // [191:160] opcode
                        r_target,            // [159:128] target
                        applied0,            // [127:96]  sanitised arg0
                        {31'b0, committed},  // [95:64]   committed + reason (packed)
                        r_context_hash,      // [63:32]   context hash
                        r_policy_hash        // [31:0]    policy hash
                    };
                    state <= S_RESPOND;
                end

                // ---------------------------------------------------------
                S_RESPOND: begin
                    ledger_wr_en    <= 1'b0;
                    resp_valid      <= 1'b1;
                    resp_committed  <= {31'b0, committed};
                    resp_reason     <= reason;
                    resp_applied0   <= applied0;
                    resp_applied1   <= applied1;
                    resp_ledger_idx <= ledger_idx;
                    act_ready       <= 1'b1;
                    state           <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
