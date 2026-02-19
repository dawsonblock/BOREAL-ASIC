`timescale 1ns/1ps
`default_nettype none
// ============================================================================
// boreal_dma.v  (sandboxed DMA master)
// - Descriptor ring in SRAM (public only). 16-byte desc = 4 words.
// - MMIO regs: HEAD, TAIL, STATUS, START, CRC (crc stub)
// - Public-only constraint: src/dst must NOT be in PRIV region
// - Deterministic fixed state machine.
// ============================================================================
`include "boreal_pkg.vh"

module boreal_dma (
    input  wire        clk,
    input  wire        rst_n,

    // MMIO
    input  wire        mmio_we,
    input  wire [7:0]  mmio_addr,
    input  wire [31:0] mmio_wdata,
    input  wire [3:0]  mmio_wstrb,
    output reg  [31:0] mmio_rdata,

    // Master port to interconnect
    output reg         m_req_valid,
    output reg         m_req_we,
    output reg  [31:0] m_req_addr,
    output reg  [31:0] m_req_wdata,
    output reg  [3:0]  m_req_wstrb,
    input  wire        m_resp_valid,
    input  wire [31:0] m_resp_rdata,
    input  wire        m_resp_err
);
    // regs
    reg [31:0] head_ptr;
    reg [31:0] tail_ptr;
    reg [31:0] status;
    reg [31:0] crc;

    // engine state
    reg [3:0]  st;
    reg [31:0] cur_desc_addr;
    reg [31:0] src_addr, dst_addr, len_bytes, ctrl;
    reg [31:0] remaining_words;
    reg [31:0] buf_word;

    wire head_ne_tail = (head_ptr != tail_ptr);

    // address sandbox check
    function [0:0] addr_is_priv;
        input [31:0] a;
        begin
            addr_is_priv = (a[31:28] == `REGN_PRIV2);
        end
    endfunction

    // MMIO map (word offsets): 0 HEAD, 1 TAIL, 2 STATUS (RO), 3 START (WO), 4 CRC (RO)
    always @(posedge clk) begin
        if (!rst_n) begin
            head_ptr <= 32'h0;
            tail_ptr <= 32'h0;
            status   <= 32'h0;
            crc      <= 32'h0;

            st <= 4'd0;
            m_req_valid <= 1'b0;
            m_req_we    <= 1'b0;
            m_req_addr  <= 32'h0;
            m_req_wdata <= 32'h0;
            m_req_wstrb <= 4'h0;

            mmio_rdata <= 32'h0;

            cur_desc_addr   <= 32'h0;
            src_addr        <= 32'h0;
            dst_addr        <= 32'h0;
            len_bytes       <= 32'h0;
            ctrl            <= 32'h0;
            remaining_words <= 32'h0;
            buf_word        <= 32'h0;
        end else begin
            // defaults
            m_req_valid <= 1'b0;

            // MMIO writes
            if (mmio_we) begin
                case (mmio_addr)
                    8'h00: head_ptr <= mmio_wdata;
                    8'h01: tail_ptr <= mmio_wdata;
                    8'h03: begin
                        if (mmio_wdata[0]) begin
                            status[`DMA_BUSY] <= 1'b1;
                            status[`DMA_DONE] <= 1'b0;
                            status[`DMA_ERR]  <= 1'b0;
                            st <= 4'd1;
                        end
                    end
                    default: ;
                endcase
            end

            // MMIO reads
            case (mmio_addr)
                8'h00: mmio_rdata <= head_ptr;
                8'h01: mmio_rdata <= tail_ptr;
                8'h02: mmio_rdata <= status;
                8'h04: mmio_rdata <= crc;
                default: mmio_rdata <= 32'h0;
            endcase

            // DMA engine FSM
            case (st)
                4'd0: begin
                    if (status[`DMA_BUSY] && head_ne_tail)
                        st <= 4'd1;
                    else if (status[`DMA_BUSY] && !head_ne_tail) begin
                        status[`DMA_BUSY] <= 1'b0;
                        status[`DMA_DONE] <= 1'b1;
                        st <= 4'd0;
                    end
                end

                // Read descriptor word0 (src)
                4'd1: begin
                    cur_desc_addr <= head_ptr;
                    m_req_valid <= 1'b1;
                    m_req_we    <= 1'b0;
                    m_req_addr  <= head_ptr;
                    st <= 4'd2;
                end
                4'd2: if (m_resp_valid) begin
                    if (m_resp_err) begin status[`DMA_ERR]<=1'b1; status[`DMA_BUSY]<=1'b0; st<=4'd0; end
                    else begin src_addr <= m_resp_rdata; st <= 4'd3; end
                end

                // Read desc word1 (dst)
                4'd3: begin
                    m_req_valid <= 1'b1; m_req_we <= 1'b0;
                    m_req_addr  <= cur_desc_addr + 32'd4;
                    st <= 4'd4;
                end
                4'd4: if (m_resp_valid) begin
                    if (m_resp_err) begin status[`DMA_ERR]<=1'b1; status[`DMA_BUSY]<=1'b0; st<=4'd0; end
                    else begin dst_addr <= m_resp_rdata; st <= 4'd5; end
                end

                // Read desc word2 (len_bytes)
                4'd5: begin
                    m_req_valid <= 1'b1; m_req_we <= 1'b0;
                    m_req_addr  <= cur_desc_addr + 32'd8;
                    st <= 4'd6;
                end
                4'd6: if (m_resp_valid) begin
                    if (m_resp_err) begin status[`DMA_ERR]<=1'b1; status[`DMA_BUSY]<=1'b0; st<=4'd0; end
                    else begin len_bytes <= m_resp_rdata; st <= 4'd7; end
                end

                // Read desc word3 (ctrl)
                4'd7: begin
                    m_req_valid <= 1'b1; m_req_we <= 1'b0;
                    m_req_addr  <= cur_desc_addr + 32'd12;
                    st <= 4'd8;
                end
                4'd8: if (m_resp_valid) begin
                    if (m_resp_err) begin status[`DMA_ERR]<=1'b1; status[`DMA_BUSY]<=1'b0; st<=4'd0; end
                    else begin
                        ctrl <= m_resp_rdata;
                        // sandbox check
                        if (addr_is_priv(src_addr) || addr_is_priv(dst_addr)) begin
                            status[`DMA_ERR]  <= 1'b1;
                            status[`DMA_BUSY] <= 1'b0;
                            st <= 4'd0;
                        end else begin
                            remaining_words <= (len_bytes >> 2);
                            st <= 4'd9;
                        end
                    end
                end

                // Transfer loop: read word
                4'd9: begin
                    if (remaining_words == 0) begin
                        head_ptr <= head_ptr + 32'd16;
                        st <= 4'd0;
                    end else begin
                        m_req_valid <= 1'b1;
                        m_req_we    <= 1'b0;
                        m_req_addr  <= src_addr;
                        st <= 4'd10;
                    end
                end
                4'd10: if (m_resp_valid) begin
                    if (m_resp_err) begin status[`DMA_ERR]<=1'b1; status[`DMA_BUSY]<=1'b0; st<=4'd0; end
                    else begin buf_word <= m_resp_rdata; st <= 4'd11; end
                end

                // write word
                4'd11: begin
                    m_req_valid <= 1'b1;
                    m_req_we    <= 1'b1;
                    m_req_addr  <= dst_addr;
                    m_req_wdata <= buf_word;
                    m_req_wstrb <= 4'hF;
                    st <= 4'd12;
                end
                4'd12: if (m_resp_valid) begin
                    if (m_resp_err) begin status[`DMA_ERR]<=1'b1; status[`DMA_BUSY]<=1'b0; st<=4'd0; end
                    else begin
                        src_addr        <= src_addr + 32'd4;
                        dst_addr        <= dst_addr + 32'd4;
                        remaining_words <= remaining_words - 1;
                        st <= 4'd9;
                    end
                end

                default: st <= 4'd0;
            endcase
        end
    end
endmodule
`default_nettype wire
