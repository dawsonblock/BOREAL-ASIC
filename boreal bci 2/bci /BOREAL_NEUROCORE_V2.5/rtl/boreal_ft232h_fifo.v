/**
 * Boreal FT232H FIFO with proper CDC
 */

module boreal_ft232h_fifo #(
    parameter DATA_WIDTH = 8,
    parameter FIFO_DEPTH = 512
)(
    input  wire                  ftdi_clk,
    input  wire                  ftdi_rst_n,
    input  wire [DATA_WIDTH-1:0] ftdi_data,
    input  wire                  ftdi_rxf_n,
    output reg                   ftdi_rd_n,
    input  wire                  ftdi_txe_n,
    output reg                   ftdi_wr_n,
    output reg  [DATA_WIDTH-1:0] ftdi_wr_data,
    input  wire                  fpga_clk,
    input  wire                  fpga_rst_n,
    output reg  [DATA_WIDTH-1:0] rx_data_out,
    output reg                   rx_valid,
    input  wire [DATA_WIDTH-1:0] tx_data_in,
    input  wire                  tx_valid,
    output wire                  tx_ready,
    output wire [15:0]           rx_fifo_level,
    output wire [15:0]           tx_fifo_level
);

    localparam ADDR_WIDTH = $clog2(FIFO_DEPTH);

    function [ADDR_WIDTH:0] bin2gray;
        input [ADDR_WIDTH:0] bin;
        bin2gray = (bin >> 1) ^ bin;
    endfunction

    function [ADDR_WIDTH:0] gray2bin;
        input [ADDR_WIDTH:0] gray;
        reg [ADDR_WIDTH:0] bin;
        integer i;
        begin
            bin[ADDR_WIDTH] = gray[ADDR_WIDTH];
            for (i = ADDR_WIDTH-1; i >= 0; i = i - 1)
                bin[i] = bin[i+1] ^ gray[i];
            gray2bin = bin;
        end
    endfunction

    reg [DATA_WIDTH-1:0] rx_fifo [0:FIFO_DEPTH-1];
    reg [ADDR_WIDTH:0] rx_wr_ptr, rx_rd_ptr;
    reg [ADDR_WIDTH:0] rx_wr_ptr_gray, rx_rd_ptr_gray;
    reg [ADDR_WIDTH:0] rx_wr_ptr_gray_sync, rx_rd_ptr_gray_sync;

    reg [DATA_WIDTH-1:0] tx_fifo [0:FIFO_DEPTH-1];
    reg [ADDR_WIDTH:0] tx_wr_ptr, tx_rd_ptr;
    reg [ADDR_WIDTH:0] tx_wr_ptr_gray, tx_rd_ptr_gray;
    reg [ADDR_WIDTH:0] tx_wr_ptr_gray_sync, tx_rd_ptr_gray_sync;

    localparam FTDI_IDLE = 2'b00, FTDI_READ = 2'b01, FTDI_WRITE = 2'b10;
    reg [1:0] ftdi_state;
    reg [3:0] wait_counter;

    always @(posedge ftdi_clk or negedge ftdi_rst_n) begin
        if (!ftdi_rst_n) begin
            ftdi_state <= FTDI_IDLE;
            ftdi_rd_n <= 1'b1;
            ftdi_wr_n <= 1'b1;
            rx_wr_ptr <= 'd0;
            tx_rd_ptr <= 'd0;
        end else begin
            case (ftdi_state)
                FTDI_IDLE: begin
                    ftdi_rd_n <= 1'b1;
                    ftdi_wr_n <= 1'b1;
                    if (!ftdi_rxf_n && (rx_wr_ptr != (rx_rd_ptr_gray_sync ^ (1'b1 << ADDR_WIDTH)))) begin
                        ftdi_state <= FTDI_READ;
                        ftdi_rd_n <= 1'b0;
                        wait_counter <= 4'd2;
                    end else if (!ftdi_txe_n && (tx_wr_ptr_gray_sync != tx_rd_ptr)) begin
                        ftdi_state <= FTDI_WRITE;
                        ftdi_wr_data <= tx_fifo[tx_rd_ptr[ADDR_WIDTH-1:0]];
                        ftdi_wr_n <= 1'b0;
                        wait_counter <= 4'd2;
                    end
                end
                FTDI_READ: begin
                    if (wait_counter > 0)
                        wait_counter <= wait_counter - 1'b1;
                    else begin
                        rx_fifo[rx_wr_ptr[ADDR_WIDTH-1:0]] <= ftdi_data;
                        rx_wr_ptr <= rx_wr_ptr + 1'b1;
                        ftdi_rd_n <= 1'b1;
                        ftdi_state <= FTDI_IDLE;
                    end
                end
                FTDI_WRITE: begin
                    if (wait_counter > 0)
                        wait_counter <= wait_counter - 1'b1;
                    else begin
                        tx_rd_ptr <= tx_rd_ptr + 1'b1;
                        ftdi_wr_n <= 1'b1;
                        ftdi_state <= FTDI_IDLE;
                    end
                end
            endcase
        end
    end

    always @(posedge ftdi_clk) begin
        rx_wr_ptr_gray <= bin2gray(rx_wr_ptr);
        tx_rd_ptr_gray <= bin2gray(tx_rd_ptr);
    end

    wire [ADDR_WIDTH:0] rx_wr_ptr_bin = gray2bin(rx_wr_ptr_gray_sync);
    assign rx_fifo_level = (rx_wr_ptr_bin >= rx_rd_ptr) ? (rx_wr_ptr_bin - rx_rd_ptr) : (FIFO_DEPTH + rx_wr_ptr_bin - rx_rd_ptr);

    always @(posedge fpga_clk or negedge fpga_rst_n) begin
        if (!fpga_rst_n) begin
            rx_rd_ptr <= 'd0;
            rx_valid <= 1'b0;
        end else begin
            rx_valid <= 1'b0;
            if (rx_rd_ptr != rx_wr_ptr_bin) begin
                rx_data_out <= rx_fifo[rx_rd_ptr[ADDR_WIDTH-1:0]];
                rx_rd_ptr <= rx_rd_ptr + 1'b1;
                rx_valid <= 1'b1;
            end
        end
    end

    wire [ADDR_WIDTH:0] tx_rd_ptr_bin = gray2bin(tx_rd_ptr_gray_sync);
    assign tx_fifo_level = (tx_wr_ptr >= tx_rd_ptr_bin) ? (tx_wr_ptr - tx_rd_ptr_bin) : (FIFO_DEPTH + tx_wr_ptr - tx_rd_ptr_bin);
    assign tx_ready = (tx_fifo_level < FIFO_DEPTH - 2);

    always @(posedge fpga_clk) begin
        if (tx_valid && tx_ready) begin
            tx_fifo[tx_wr_ptr[ADDR_WIDTH-1:0]] <= tx_data_in;
            tx_wr_ptr <= tx_wr_ptr + 1'b1;
        end
    end

    always @(posedge fpga_clk) begin
        tx_wr_ptr_gray <= bin2gray(tx_wr_ptr);
        rx_rd_ptr_gray <= bin2gray(rx_rd_ptr);
    end

    always @(posedge ftdi_clk) begin
        tx_wr_ptr_gray_sync <= tx_wr_ptr_gray;
        tx_wr_ptr_gray_sync <= tx_wr_ptr_gray_sync;
    end

    always @(posedge fpga_clk) begin
        rx_wr_ptr_gray_sync <= rx_wr_ptr_gray;
        rx_wr_ptr_gray_sync <= rx_wr_ptr_gray_sync;
        tx_rd_ptr_gray_sync <= tx_rd_ptr_gray;
        tx_rd_ptr_gray_sync <= tx_rd_ptr_gray_sync;
    end
endmodule
