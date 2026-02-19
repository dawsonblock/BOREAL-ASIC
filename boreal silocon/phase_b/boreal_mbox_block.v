`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_mbox_block.v  (Phase-B mailbox block)
// Three logical mailboxes in one module:
//   1. AI input (4 words, CPU-writable, VM-readable)
//   2. Action request (16 words, VM-writable, Gate-readable + valid/consume)
//   3. Gate response (5 words, Gate-writable, VM-readable + valid/ack)
// MMIO port for CPU access. Internal ports for VM and Gate.
// ============================================================================
`include "boreal_pkg.vh"

module boreal_mbox_block (
    input  wire        clk,
    input  wire        rst_n,

    // CPU/MMIO port
    input  wire        mmio_we,
    input  wire [7:0]  mmio_addr_word,
    input  wire [31:0] mmio_wdata,
    input  wire [3:0]  mmio_wstrb,
    output reg  [31:0] mmio_rdata,

    // AI input words (output to VM)
    output reg [127:0] mb_ai,

    // VM -> request mailbox (internal write port)
    input  wire        vm_req_we,
    input  wire [3:0]  vm_req_widx,
    input  wire [31:0] vm_req_wdata,
    input  wire        vm_req_valid_set,
    input  wire        vm_resp_ack,

    // Gate -> request mailbox (internal read port)
    output reg         gate_req_valid,
    output wire [31:0] gate_req_words_0,
    output wire [31:0] gate_req_words_1,
    output wire [31:0] gate_req_words_2,
    output wire [31:0] gate_req_words_3,
    output wire [31:0] gate_req_words_4,
    output wire [31:0] gate_req_words_5,
    output wire [31:0] gate_req_words_6,
    output wire [31:0] gate_req_words_7,
    output wire [31:0] gate_req_words_8,
    output wire [31:0] gate_req_words_9,
    output wire [31:0] gate_req_words_10,
    output wire [31:0] gate_req_words_11,
    output wire [31:0] gate_req_words_12,
    output wire [31:0] gate_req_words_13,
    output wire [31:0] gate_req_words_14,
    output wire [31:0] gate_req_words_15,
    input  wire        gate_req_consume,

    // Gate -> response mailbox (internal write port)
    input  wire        gate_resp_we,
    input  wire [2:0]  gate_resp_widx,
    input  wire [31:0] gate_resp_wdata,
    input  wire        gate_resp_valid_set,

    // Response mailbox -> VM (read port)
    output reg         gate_resp_valid,
    output wire [31:0] gate_resp_words_0,
    output wire [31:0] gate_resp_words_1,
    output wire [31:0] gate_resp_words_2,
    output wire [31:0] gate_resp_words_3,
    output wire [31:0] gate_resp_words_4
);
    // -----------------------------------------------------------------------
    // AI input mailbox (4 words)
    // -----------------------------------------------------------------------
    reg [31:0] ai_words [0:3];

    integer i;
    initial begin
        for (i = 0; i < 4; i = i + 1) ai_words[i] = 32'h0;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            mb_ai <= 128'h0;
        end else begin
            // CPU writes to AI words
            if (mmio_we && mmio_addr_word <= 8'h03)
                ai_words[mmio_addr_word[1:0]] <= mmio_wdata;

            // Continuously latch AI words into packed output
            mb_ai <= {ai_words[3], ai_words[2], ai_words[1], ai_words[0]};
        end
    end

    // -----------------------------------------------------------------------
    // Action request mailbox (16 words + valid flag)
    // -----------------------------------------------------------------------
    reg [31:0] req_words [0:15];

    integer j;
    initial begin
        for (j = 0; j < 16; j = j + 1) req_words[j] = 32'h0;
    end

    assign gate_req_words_0  = req_words[0];
    assign gate_req_words_1  = req_words[1];
    assign gate_req_words_2  = req_words[2];
    assign gate_req_words_3  = req_words[3];
    assign gate_req_words_4  = req_words[4];
    assign gate_req_words_5  = req_words[5];
    assign gate_req_words_6  = req_words[6];
    assign gate_req_words_7  = req_words[7];
    assign gate_req_words_8  = req_words[8];
    assign gate_req_words_9  = req_words[9];
    assign gate_req_words_10 = req_words[10];
    assign gate_req_words_11 = req_words[11];
    assign gate_req_words_12 = req_words[12];
    assign gate_req_words_13 = req_words[13];
    assign gate_req_words_14 = req_words[14];
    assign gate_req_words_15 = req_words[15];

    always @(posedge clk) begin
        if (!rst_n) begin
            gate_req_valid <= 1'b0;
        end else begin
            // VM writes request words
            if (vm_req_we)
                req_words[vm_req_widx] <= vm_req_wdata;

            // VM sets valid
            if (vm_req_valid_set)
                gate_req_valid <= 1'b1;

            // Gate consumes request
            if (gate_req_consume)
                gate_req_valid <= 1'b0;
        end
    end

    // -----------------------------------------------------------------------
    // Gate response mailbox (5 words + valid flag)
    // -----------------------------------------------------------------------
    reg [31:0] resp_words [0:4];

    integer k;
    initial begin
        for (k = 0; k < 5; k = k + 1) resp_words[k] = 32'h0;
    end

    assign gate_resp_words_0 = resp_words[0];
    assign gate_resp_words_1 = resp_words[1];
    assign gate_resp_words_2 = resp_words[2];
    assign gate_resp_words_3 = resp_words[3];
    assign gate_resp_words_4 = resp_words[4];

    always @(posedge clk) begin
        if (!rst_n) begin
            gate_resp_valid <= 1'b0;
        end else begin
            // Gate writes response words
            if (gate_resp_we)
                resp_words[gate_resp_widx] <= gate_resp_wdata;

            // Gate sets valid
            if (gate_resp_valid_set)
                gate_resp_valid <= 1'b1;

            // VM acks response
            if (vm_resp_ack)
                gate_resp_valid <= 1'b0;
        end
    end

    // -----------------------------------------------------------------------
    // MMIO read mux
    // -----------------------------------------------------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            mmio_rdata <= 32'h0;
        end else begin
            if (mmio_addr_word <= 8'h03)
                mmio_rdata <= ai_words[mmio_addr_word[1:0]];
            else if (mmio_addr_word == `MBOX_REQ_VALID)
                mmio_rdata <= {31'h0, gate_req_valid};
            else if (mmio_addr_word >= `MBOX_REQ_W0 && mmio_addr_word < (`MBOX_REQ_W0 + 8'd16))
                mmio_rdata <= req_words[mmio_addr_word - `MBOX_REQ_W0];
            else if (mmio_addr_word == `MBOX_RESP_VALID)
                mmio_rdata <= {31'h0, gate_resp_valid};
            else if (mmio_addr_word >= `MBOX_RESP_W0 && mmio_addr_word < (`MBOX_RESP_W0 + 8'd5))
                mmio_rdata <= resp_words[mmio_addr_word - `MBOX_RESP_W0];
            else
                mmio_rdata <= 32'h0;
        end
    end
endmodule
`default_nettype wire
