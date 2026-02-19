// ============================================================================
// Boreal SoC - DMA Ring Engine
// ============================================================================
// Descriptor-ring DMA with CRC-32 computation.  Supports single-word
// copy operations between SRAM addresses.  Descriptors are stored in
// a 16-entry ring buffer.
// ============================================================================
`timescale 1ns / 1ps
`include "boreal_pkg.v"

module boreal_dma_ring #(
    parameter RING_DEPTH     = 16,
    parameter RING_DEPTH_LOG = 4
)(
    input  wire        clk,
    input  wire        rst_n,

    // --- MMIO slave interface ---
    input  wire        sel,
    input  wire        wr,
    input  wire [31:0] addr,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,
    output reg         ack,

    // --- SRAM DMA port ---
    output reg         mem_sel,
    output reg         mem_wr,
    output reg  [ 9:0] mem_addr,
    output reg  [31:0] mem_wdata,
    input  wire [31:0] mem_rdata,
    input  wire        mem_ack
);

    // MMIO register offsets
    localparam OFF_HEAD   = 8'h00;
    localparam OFF_TAIL   = 8'h04;
    localparam OFF_STATUS = 8'h08;
    localparam OFF_START  = 8'h0C;
    localparam OFF_CRC    = 8'h10;

    // Descriptor ring – each entry: {src_addr[31:0], dst_addr[31:0], length[31:0]}
    reg [31:0] desc_src  [0:RING_DEPTH-1];
    reg [31:0] desc_dst  [0:RING_DEPTH-1];
    reg [31:0] desc_len  [0:RING_DEPTH-1];

    // Control registers
    reg [RING_DEPTH_LOG-1:0] head;
    reg [RING_DEPTH_LOG-1:0] tail;
    reg        busy;
    reg        done;
    reg        error;
    reg [31:0] crc_reg;

    // DMA engine state
    localparam DS_IDLE    = 3'd0;
    localparam DS_FETCH   = 3'd1;
    localparam DS_READ    = 3'd2;
    localparam DS_WAIT_R  = 3'd3;
    localparam DS_WRITE   = 3'd4;
    localparam DS_WAIT_W  = 3'd5;
    localparam DS_NEXT    = 3'd6;

    reg [2:0]  dma_state;
    reg [31:0] cur_src, cur_dst, cur_len;
    reg [31:0] xfer_count;
    reg [31:0] read_data_lat;

    // -----------------------------------------------------------------------
    // CRC-32 (IEEE 802.3 polynomial, simplified bit-serial per word)
    // -----------------------------------------------------------------------
    function [31:0] crc32_word;
        input [31:0] crc_in;
        input [31:0] data;
        reg [31:0] c;
        integer i;
        begin
            c = crc_in ^ data;
            for (i = 0; i < 32; i = i + 1) begin
                if (c[0])
                    c = {1'b0, c[31:1]} ^ 32'hEDB88320;
                else
                    c = {1'b0, c[31:1]};
            end
            crc32_word = c;
        end
    endfunction

    // -----------------------------------------------------------------------
    // MMIO interface
    // -----------------------------------------------------------------------
    wire [7:0] reg_off = addr[7:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tail <= {RING_DEPTH_LOG{1'b0}};
        end else if (sel && wr && reg_off == OFF_TAIL) begin
            tail <= wdata[RING_DEPTH_LOG-1:0];
        end
    end

    always @(*) begin
        rdata = 32'h0;
        ack   = sel;
        if (sel && !wr) begin
            case (reg_off)
                OFF_HEAD:   rdata = {28'b0, head};
                OFF_TAIL:   rdata = {28'b0, tail};
                OFF_STATUS: rdata = {29'b0, error, done, busy};
                OFF_CRC:    rdata = crc_reg;
                default:    rdata = 32'h0;
            endcase
        end
    end

    // -----------------------------------------------------------------------
    // DMA engine FSM
    // -----------------------------------------------------------------------
    wire start_pulse = sel && wr && (reg_off == OFF_START) && wdata[0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dma_state  <= DS_IDLE;
            head       <= {RING_DEPTH_LOG{1'b0}};
            busy       <= 1'b0;
            done       <= 1'b0;
            error      <= 1'b0;
            crc_reg    <= 32'hFFFF_FFFF;
            mem_sel    <= 1'b0;
            mem_wr     <= 1'b0;
            mem_addr   <= 10'd0;
            mem_wdata  <= 32'd0;
            cur_src    <= 32'd0;
            cur_dst    <= 32'd0;
            cur_len    <= 32'd0;
            xfer_count <= 32'd0;
            read_data_lat <= 32'd0;
        end else begin
            case (dma_state)
                DS_IDLE: begin
                    mem_sel <= 1'b0;
                    if (sel && wr && reg_off == OFF_HEAD)
                        head <= wdata[RING_DEPTH_LOG-1:0];
                    if (start_pulse && head != tail) begin
                        busy      <= 1'b1;
                        done      <= 1'b0;
                        error     <= 1'b0;
                        crc_reg   <= 32'hFFFF_FFFF;
                        dma_state <= DS_FETCH;
                    end
                end

                DS_FETCH: begin
                    if (head == tail) begin
                        // Ring empty – transfer complete
                        busy      <= 1'b0;
                        done      <= 1'b1;
                        crc_reg   <= ~crc_reg;
                        dma_state <= DS_IDLE;
                    end else begin
                        cur_src    <= desc_src[head];
                        cur_dst    <= desc_dst[head];
                        cur_len    <= desc_len[head];
                        xfer_count <= 32'd0;
                        dma_state  <= DS_READ;
                    end
                end

                DS_READ: begin
                    if (xfer_count >= cur_len) begin
                        head      <= head + 1;
                        dma_state <= DS_FETCH;
                    end else begin
                        mem_sel   <= 1'b1;
                        mem_wr    <= 1'b0;
                        mem_addr  <= cur_src[11:2] + xfer_count[9:0];
                        dma_state <= DS_WAIT_R;
                    end
                end

                DS_WAIT_R: begin
                    if (mem_ack) begin
                        read_data_lat <= mem_rdata;
                        crc_reg       <= crc32_word(crc_reg, mem_rdata);
                        mem_sel       <= 1'b0;
                        dma_state     <= DS_WRITE;
                    end
                end

                DS_WRITE: begin
                    mem_sel   <= 1'b1;
                    mem_wr    <= 1'b1;
                    mem_addr  <= cur_dst[11:2] + xfer_count[9:0];
                    mem_wdata <= read_data_lat;
                    dma_state <= DS_WAIT_W;
                end

                DS_WAIT_W: begin
                    if (mem_ack) begin
                        mem_sel    <= 1'b0;
                        mem_wr     <= 1'b0;
                        xfer_count <= xfer_count + 1;
                        dma_state  <= DS_READ;
                    end
                end

                default: dma_state <= DS_IDLE;
            endcase
        end
    end

endmodule
