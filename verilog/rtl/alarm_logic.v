// SPDX-License-Identifier: Apache-2.0
// SenseEdge - Alarm & Interrupt Logic
// Monitors classification results and generates alarms
// Configurable confidence threshold and consecutive fault counter

`default_nettype none

module alarm_logic (
    input  wire       clk,
    input  wire       rst,

    // Classification input
    input  wire       classification_done,  // Pulse on new result
    input  wire [1:0] class_id,
    input  wire [7:0] confidence,

    // Configuration
    input  wire [7:0] alarm_threshold,      // Min confidence to count as fault
    input  wire [3:0] fault_count_cfg,      // Consecutive faults before alarm

    // Outputs
    output reg        alarm_active,         // Persistent alarm flag
    output reg        alarm_irq,            // Single-cycle IRQ pulse
    output reg  [1:0] last_fault_class      // Which fault class triggered alarm
);

    // Class 0 = healthy, classes 1-3 = fault conditions
    reg [3:0] consec_fault_count;

    always @(posedge clk) begin
        if (rst) begin
            alarm_active      <= 1'b0;
            alarm_irq         <= 1'b0;
            consec_fault_count <= 4'd0;
            last_fault_class  <= 2'd0;
        end else begin
            alarm_irq <= 1'b0;  // Default: single-cycle pulse

            if (classification_done) begin
                if (class_id != 2'd0 && confidence >= alarm_threshold) begin
                    // Fault detected with sufficient confidence
                    if (consec_fault_count < 4'd15) begin
                        consec_fault_count <= consec_fault_count + 4'd1;
                    end
                    last_fault_class <= class_id;

                    // Check if we've hit the threshold
                    if (consec_fault_count >= fault_count_cfg && !alarm_active) begin
                        alarm_active <= 1'b1;
                        alarm_irq    <= 1'b1;
                    end
                end else begin
                    // Healthy classification or low confidence - reset counter
                    consec_fault_count <= 4'd0;
                    // Clear alarm when machine returns to healthy
                    if (class_id == 2'd0 && confidence >= alarm_threshold) begin
                        alarm_active <= 1'b0;
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire
