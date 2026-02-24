/*
 * Boreal ADS1299 SPI Interface
 * Handles communication with the ADS1299 EEG Front-End
 */

module boreal_ads1299_spi (
    input  wire         clk_100m,
    input  wire         rst_n,
    input  wire         bite_switch_n,
    input  wire         drdy_n,
    input  wire         miso,
    output reg          sclk,
    output reg          cs_n,
    output reg  [127:0] eeg_channels_out, // Simplified to 128 bits for top-level
    output reg          data_valid
);

    // Simple SPI state machine to shift in data when drdy_n goes low
    reg [7:0] bit_counter;
    reg [127:0] shift_reg;
    reg [2:0] state;

    localparam IDLE = 3'd0;
    localparam READ = 3'd1;
    localparam DONE = 3'd2;

    always @(posedge clk_100m or negedge rst_n) begin
        if (!rst_n) begin
            sclk <= 0;
            cs_n <= 1;
            eeg_channels_out <= 0;
            data_valid <= 0;
            bit_counter <= 0;
            shift_reg <= 0;
            state <= IDLE;
        end else if (!bite_switch_n) begin
            cs_n <= 1;
            data_valid <= 0;
            state <= IDLE;
        end else begin
            case (state)
                IDLE: begin
                    data_valid <= 0;
                    if (!drdy_n) begin
                        cs_n <= 0;
                        bit_counter <= 0;
                        state <= READ;
                    end
                end
                
                READ: begin
                    sclk <= ~sclk;
                    if (sclk) begin // Falling edge of SCLK
                        shift_reg <= {shift_reg[126:0], miso};
                        bit_counter <= bit_counter + 1;
                        if (bit_counter == 127) begin
                            state <= DONE;
                        end
                    end
                end
                
                DONE: begin
                    cs_n <= 1;
                    eeg_channels_out <= shift_reg;
                    data_valid <= 1;
                    state <= IDLE;
                end
                
                default: state <= IDLE;
            endcase
        end
    end

endmodule
