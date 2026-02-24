/*
 * Boreal BCI Symbolic Intent Mapper
 * 
 * Purpose:
 * Converts continuous, continuous 2D BCI cursor vectors into discrete, debounced
 * symbolic triggers (e.g., D-Pad, actions) via a "stare-to-select" temporal integration.
 * Prevents accidental neurological misfires from triggering system actions.
 */

module boreal_symbolic_mapper #(
    // Constants for Thresholds (16-bit Signed Space: -32768 to 32767)
    // We arbitrarily set boundaries at +/- 10000 
    parameter signed [15:0] THRESHOLD_POS = 16'd10000,
    parameter signed [15:0] THRESHOLD_NEG = -16'd10000,
    
    // Dwell Time: The number of consecutive inference cycles the vector must be 
    // held past the threshold to confirm selection.
    // 10 cycles ≈ small 0.1s lag assuming 100Hz BCI refresh rate
    parameter DWELL_TIME_CYCLES = 10,

    // Refractory Period: The number of cycles the system ignores ALL inputs 
    // after dropping a trigger pulse, preventing bouncing.
    parameter COOLDOWN_CYCLES = 50
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        enable,
    input  wire        valid_in,

    // Continuous smooth vector inputs (from Boreal Active Inference)
    input  wire signed [15:0] cursor_x,
    input  wire signed [15:0] cursor_y,

    // Discrete 4-Bit Output Trigger Vector: [UP, DOWN, LEFT, RIGHT]
    // Asserted HIGH for a SINGLE clock cycle upon successful integration
    output reg [3:0]   symbol_trigger,
    output reg         valid_out
);

    // Internal Register State Machines
    reg [15:0] dwell_counter;
    reg [15:0] cooldown_counter;
    reg [2:0]  state;

    // States
    localparam STATE_IDLE      = 3'b000;
    localparam STATE_INTEGRATE = 3'b001;
    localparam STATE_TRIGGER   = 3'b010;
    localparam STATE_COOLDOWN  = 3'b011;

    // Internal boundary flags (combinatorial limits)
    wire cross_right = (cursor_x >= THRESHOLD_POS);
    wire cross_left  = (cursor_x <= THRESHOLD_NEG);
    wire cross_up    = (cursor_y >= THRESHOLD_POS);
    wire cross_down  = (cursor_y <= THRESHOLD_NEG);

    // Identify which discrete symbol is currently the "target" being stared at
    // Only one symbol is allowed at a time (cardinal directions only)
    reg [3:0] active_target;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dwell_counter <= 0;
            cooldown_counter <= 0;
            state <= STATE_IDLE;
            active_target <= 4'b0000;
            symbol_trigger <= 4'b0000;
            valid_out <= 0;
        end else if (enable) begin
            
            // Generate clean 1-cycle trigger pulses
            symbol_trigger <= 4'b0000; 
            valid_out <= 0;

            if (valid_in) begin
                case (state)
                    STATE_IDLE: begin
                        // Check if cursor crossed any cardinal boundary
                        if (cross_up)    active_target <= 4'b1000;
                        else if (cross_down)  active_target <= 4'b0100;
                        else if (cross_left)  active_target <= 4'b0010;
                        else if (cross_right) active_target <= 4'b0001;
                        else active_target <= 4'b0000;

                        if (active_target != 4'b0000) begin
                            dwell_counter <= 1;
                            state <= STATE_INTEGRATE;
                        end
                    end

                    STATE_INTEGRATE: begin
                        // Check if the user is maintaining the SPECIFIC intent
                        if (
                            (active_target == 4'b1000 && cross_up)    ||
                            (active_target == 4'b0100 && cross_down)  ||
                            (active_target == 4'b0010 && cross_left)  ||
                            (active_target == 4'b0001 && cross_right)
                        ) begin
                            // Maintain intent, increment dwell integration
                            // Emit trigger immediately when dwell threshold is reached
                            if (dwell_counter >= (DWELL_TIME_CYCLES - 1)) begin
                                symbol_trigger <= active_target;
                                valid_out <= 1;
                                cooldown_counter <= 0;
                                state <= STATE_COOLDOWN;
                            end else begin
                                dwell_counter <= dwell_counter + 1;
                            end
                        end else begin
                            // User slipped off the boundary, intent lost
                            active_target <= 4'b0000;
                            dwell_counter <= 0;
                            state <= STATE_IDLE;
                        end
                    end

                    STATE_COOLDOWN: begin
                        // Ignore all crossing logic until refractory period ends
                        if (cooldown_counter >= COOLDOWN_CYCLES) begin
                            state <= STATE_IDLE;
                            active_target <= 4'b0000;
                        end else begin
                            cooldown_counter <= cooldown_counter + 1;
                        end
                    end

                    default: state <= STATE_IDLE;
                endcase
            end
        end else begin
            symbol_trigger <= 4'b0000;
            valid_out <= 0;
        end
    end

endmodule
