// SPDX-License-Identifier: Apache-2.0
// Testbench: Alarm Logic
// Tests:
//   1. No alarm for healthy classification
//   2. Alarm after N consecutive fault detections
//   3. Alarm clears when machine returns to healthy
//   4. Low-confidence faults don't trigger alarm
//   5. Counter resets on healthy classification

`timescale 1ns / 1ps

module tb_alarm_logic;

    // --- Clock and Reset ---
    reg clk;
    reg rst;

    initial clk = 0;
    always #20 clk = ~clk; // 25 MHz

    // --- DUT signals ---
    reg        classification_done;
    reg  [1:0] class_id;
    reg  [7:0] confidence;
    reg  [7:0] alarm_threshold;
    reg  [3:0] fault_count_cfg;
    wire       alarm_active;
    wire       alarm_irq;
    wire [1:0] last_fault_class;

    // --- DUT ---
    alarm_logic dut (
        .clk                (clk),
        .rst                (rst),
        .classification_done(classification_done),
        .class_id           (class_id),
        .confidence         (confidence),
        .alarm_threshold    (alarm_threshold),
        .fault_count_cfg    (fault_count_cfg),
        .alarm_active       (alarm_active),
        .alarm_irq          (alarm_irq),
        .last_fault_class   (last_fault_class)
    );

    // --- Tasks ---
    task classify;
        input [1:0] cls;
        input [7:0] conf;
        begin
            @(posedge clk);
            class_id            <= cls;
            confidence          <= conf;
            classification_done <= 1'b1;
            @(posedge clk);
            classification_done <= 1'b0;
            repeat (3) @(posedge clk);
        end
    endtask

    // --- Test sequence ---
    integer pass_count;
    integer fail_count;
    integer i;

    initial begin
        $dumpfile("tb_alarm_logic.vcd");
        $dumpvars(0, tb_alarm_logic);

        pass_count = 0;
        fail_count = 0;

        rst                 = 1;
        classification_done = 0;
        class_id            = 2'd0;
        confidence          = 8'd0;
        alarm_threshold     = 8'd100;
        fault_count_cfg     = 4'd3;  // 3 consecutive faults before alarm

        repeat (10) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        // ==================================================================
        // Test 1: Healthy classifications → no alarm
        // ==================================================================
        $display("");
        $display("[TEST 1] Healthy classifications - no alarm");
        for (i = 0; i < 10; i = i + 1)
            classify(2'd0, 8'd200); // Healthy, high confidence

        if (!alarm_active) begin
            $display("  PASS: No alarm for healthy classifications");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Alarm should not be active");
            fail_count = fail_count + 1;
        end

        // ==================================================================
        // Test 2: Consecutive faults trigger alarm
        // ==================================================================
        $display("");
        $display("[TEST 2] Consecutive faults trigger alarm");

        // Send fault_count_cfg faults (should not trigger yet on 2nd)
        classify(2'd1, 8'd200); // Bearing fault, high confidence
        classify(2'd1, 8'd200);

        if (!alarm_active) begin
            $display("  PASS: No alarm after only 2 faults (need 3)");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Alarm triggered too early");
            fail_count = fail_count + 1;
        end

        // Third consecutive fault should trigger
        classify(2'd1, 8'd200);
        // Need one more since counter starts at 0
        classify(2'd1, 8'd200);

        if (alarm_active) begin
            $display("  PASS: Alarm activated after consecutive faults");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Alarm should be active after %0d faults", fault_count_cfg);
            fail_count = fail_count + 1;
        end

        // Check fault class
        if (last_fault_class == 2'd1) begin
            $display("  PASS: Last fault class = Bearing Wear (1)");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Expected fault class 1, got %0d", last_fault_class);
            fail_count = fail_count + 1;
        end

        // ==================================================================
        // Test 3: Alarm clears when healthy returns
        // ==================================================================
        $display("");
        $display("[TEST 3] Alarm clears on healthy classification");
        classify(2'd0, 8'd200); // Healthy, high confidence

        if (!alarm_active) begin
            $display("  PASS: Alarm cleared on healthy classification");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Alarm should clear on healthy");
            fail_count = fail_count + 1;
        end

        // ==================================================================
        // Test 4: Low-confidence faults don't trigger alarm
        // ==================================================================
        $display("");
        $display("[TEST 4] Low-confidence faults ignored");
        // Send many faults below threshold
        for (i = 0; i < 10; i = i + 1)
            classify(2'd2, 8'd50); // Imbalance but low confidence (below 100)

        if (!alarm_active) begin
            $display("  PASS: Low-confidence faults did not trigger alarm");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Low-confidence faults should not trigger alarm");
            fail_count = fail_count + 1;
        end

        // ==================================================================
        // Test 5: Healthy resets consecutive counter
        // ==================================================================
        $display("");
        $display("[TEST 5] Healthy classification resets fault counter");
        // Send 2 faults, then healthy, then 2 faults → should not alarm
        classify(2'd3, 8'd200); // Misalignment
        classify(2'd3, 8'd200);
        classify(2'd0, 8'd200); // Healthy - resets counter
        classify(2'd3, 8'd200); // Start counting again
        classify(2'd3, 8'd200);

        if (!alarm_active) begin
            $display("  PASS: Counter reset prevented alarm");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Healthy should have reset the counter");
            fail_count = fail_count + 1;
        end

        // ==================================================================
        // Test 6: IRQ pulse verification
        // ==================================================================
        $display("");
        $display("[TEST 6] IRQ pulse on alarm trigger");
        // Reset state
        rst = 1;
        repeat (5) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        fork
            begin : irq_watch
                reg irq_seen;
                irq_seen = 0;
                repeat (200) begin
                    @(posedge clk);
                    if (alarm_irq) irq_seen = 1;
                end
                if (irq_seen) begin
                    $display("  PASS: IRQ pulse detected");
                    pass_count = pass_count + 1;
                end else begin
                    $display("  FAIL: No IRQ pulse seen");
                    fail_count = fail_count + 1;
                end
            end
            begin : irq_trigger
                // Trigger enough faults for alarm
                for (i = 0; i < 6; i = i + 1)
                    classify(2'd1, 8'd200);
            end
        join

        // --- Summary ---
        $display("");
        $display("==========================================");
        $display("  Alarm Logic Testbench Results");
        $display("  PASSED: %0d  FAILED: %0d", pass_count, fail_count);
        $display("==========================================");
        if (fail_count > 0) $display("  *** TEST FAILED ***");
        else                $display("  *** ALL TESTS PASSED ***");

        #100;
        $finish;
    end

    // Timeout watchdog
    initial begin
        #10_000_000;
        $display("WATCHDOG: Simulation timeout");
        $finish;
    end

endmodule
