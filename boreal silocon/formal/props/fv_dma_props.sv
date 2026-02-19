// ============================================================================
// Formal properties for boreal_dma_ring
// ============================================================================

module fv_dma_props #(
    parameter RING_DEPTH     = 16,
    parameter RING_DEPTH_LOG = 4
)(
    input wire        clk,
    input wire        rst_n,
    input wire [2:0]  dma_state,
    input wire        mem_sel,
    input wire        mem_wr,
    input wire [9:0]  mem_addr,
    input wire        busy,
    input wire        done,
    input wire [RING_DEPTH_LOG-1:0] head,
    input wire [RING_DEPTH_LOG-1:0] tail
);

    localparam DS_IDLE    = 3'd0;
    localparam DS_FETCH   = 3'd1;
    localparam DS_READ    = 3'd2;
    localparam DS_WAIT_R  = 3'd3;
    localparam DS_WRITE   = 3'd4;
    localparam DS_WAIT_W  = 3'd5;

    // P1: mem_sel only in read/write states
    always @(posedge clk)
        if (rst_n && mem_sel) assert (
            dma_state == DS_READ  || dma_state == DS_WAIT_R ||
            dma_state == DS_WRITE || dma_state == DS_WAIT_W
        );

    // P2: mem_wr only in write states
    always @(posedge clk)
        if (rst_n && mem_wr) assert (
            dma_state == DS_WRITE || dma_state == DS_WAIT_W
        );

    // P3: head and tail within ring depth
    always @(posedge clk) begin
        if (rst_n) begin
            assert (head < RING_DEPTH);
            assert (tail < RING_DEPTH);
        end
    end

    // P4: IDLE → not busy (unless ring non-empty from start_pulse same cycle)
    always @(posedge clk)
        if (rst_n && dma_state == DS_IDLE) assert (!busy);

endmodule
