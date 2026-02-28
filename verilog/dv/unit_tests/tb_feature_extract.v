// SPDX-License-Identifier: Apache-2.0
// Testbench: Feature Extraction Engine
// Tests:
//   1. Known magnitude spectrum → verify band energies
//   2. Single peak → verify peak detection
//   3. All-zero input → verify zero features

`timescale 1ns / 1ps

module tb_feature_extract;

    // --- Clock and Reset ---
    reg clk;
    reg rst;

    initial clk = 0;
    always #20 clk = ~clk; // 25 MHz

    // --- DUT signals ---
    reg         start;
    wire [4:0]  mag_addr;
    reg  [15:0] mag_in;
    wire        done;
    wire [7:0]  feature_out;
    reg  [2:0]  feature_addr;
    wire        busy;

    // --- Magnitude memory ---
    reg [15:0] mag_mem [0:31];

    always @(*) begin
        mag_in = mag_mem[mag_addr];
    end

    // --- DUT ---
    feature_extract dut (
        .clk         (clk),
        .rst         (rst),
        .start       (start),
        .mag_addr    (mag_addr),
        .mag_in      (mag_in),
        .done        (done),
        .feature_out (feature_out),
        .feature_addr(feature_addr),
        .busy        (busy)
    );

    // --- Tasks ---
    task run_extraction;
        begin
            @(posedge clk);
            start <= 1;
            @(posedge clk);
            start <= 0;

            begin : wait_extract_done
                integer wait_cnt;
                wait_cnt = 0;
                while (done !== 1'b1 && wait_cnt < 10000) begin
                    @(posedge clk);
                    wait_cnt = wait_cnt + 1;
                end
                if (wait_cnt >= 10000)
                    $display("  TIMEOUT: Feature extraction did not complete");
            end

            repeat (5) @(posedge clk);
        end
    endtask

    task display_features;
        integer f;
        begin
            for (f = 0; f < 8; f = f + 1) begin
                feature_addr = f[2:0];
                repeat (2) @(posedge clk);
                case (f)
                    0: $display("    [0] Band Low     = %3d", feature_out);
                    1: $display("    [1] Band Mid-Low = %3d", feature_out);
                    2: $display("    [2] Band Mid-Hi  = %3d", feature_out);
                    3: $display("    [3] Band High    = %3d", feature_out);
                    4: $display("    [4] Peak Freq    = %3d", feature_out);
                    5: $display("    [5] Peak Mag     = %3d", feature_out);
                    6: $display("    [6] Spect Cent   = %3d", feature_out);
                    7: $display("    [7] Total Energy = %3d", feature_out);
                endcase
            end
        end
    endtask

    task read_feature;
        input [2:0] addr;
        output [7:0] val;
        begin
            feature_addr = addr;
            repeat (2) @(posedge clk);
            val = feature_out;
        end
    endtask

    // --- Test sequence ---
    integer pass_count;
    integer fail_count;
    integer i;
    reg [7:0] feat_val;

    initial begin
        $dumpfile("tb_feature_extract.vcd");
        $dumpvars(0, tb_feature_extract);

        pass_count = 0;
        fail_count = 0;

        rst   = 1;
        start = 0;
        feature_addr = 0;

        repeat (10) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        // ==================================================================
        // Test 1: Energy concentrated in low band (bins 1-4)
        // ==================================================================
        $display("");
        $display("[TEST 1] Low-band energy");
        for (i = 0; i < 32; i = i + 1)
            mag_mem[i] = 16'd0;
        // Set high values in bins 1-4
        mag_mem[1] = 16'd5000;
        mag_mem[2] = 16'd5000;
        mag_mem[3] = 16'd5000;
        mag_mem[4] = 16'd5000;

        run_extraction;
        display_features;

        // Band low should be non-zero
        read_feature(3'd0, feat_val);
        if (feat_val > 8'd0) begin
            $display("  PASS: Low band energy = %0d (non-zero)", feat_val);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Low band energy should be non-zero");
            fail_count = fail_count + 1;
        end

        // Other bands should be zero
        read_feature(3'd3, feat_val);
        if (feat_val == 8'd0) begin
            $display("  PASS: High band energy = 0 (correct)");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: High band energy should be 0, got %0d", feat_val);
            fail_count = fail_count + 1;
        end

        // ==================================================================
        // Test 2: Single peak at bin 15
        // ==================================================================
        $display("");
        $display("[TEST 2] Single peak at bin 15");
        for (i = 0; i < 32; i = i + 1)
            mag_mem[i] = 16'd100;
        mag_mem[15] = 16'd10000;

        run_extraction;
        display_features;

        // Peak frequency should point to bin 15 (or nearby due to pipeline latency)
        // Feature[4] = peak_bin * 8
        read_feature(3'd4, feat_val);
        if (feat_val >= 8'd112 && feat_val <= 8'd120) begin
            $display("  PASS: Peak frequency index = %0d (bin %0d)", feat_val, feat_val / 8);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Expected peak index 112-120, got %0d", feat_val);
            fail_count = fail_count + 1;
        end

        // Peak magnitude should be high
        read_feature(3'd5, feat_val);
        if (feat_val > 8'd30) begin
            $display("  PASS: Peak magnitude = %0d (high)", feat_val);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Peak magnitude should be high, got %0d", feat_val);
            fail_count = fail_count + 1;
        end

        // ==================================================================
        // Test 3: Energy in high band (bins 21-31)
        // ==================================================================
        $display("");
        $display("[TEST 3] High-band energy");
        for (i = 0; i < 32; i = i + 1)
            mag_mem[i] = 16'd0;
        for (i = 21; i < 32; i = i + 1)
            mag_mem[i] = 16'd4000;

        run_extraction;
        display_features;

        read_feature(3'd3, feat_val);
        if (feat_val > 8'd0) begin
            $display("  PASS: High band energy = %0d (non-zero)", feat_val);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: High band energy should be non-zero");
            fail_count = fail_count + 1;
        end

        read_feature(3'd0, feat_val);
        if (feat_val == 8'd0) begin
            $display("  PASS: Low band energy = 0 (correct)");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Low band energy should be 0, got %0d", feat_val);
            fail_count = fail_count + 1;
        end

        // ==================================================================
        // Test 4: All-zero input
        // ==================================================================
        $display("");
        $display("[TEST 4] All-zero input");
        for (i = 0; i < 32; i = i + 1)
            mag_mem[i] = 16'd0;

        run_extraction;
        display_features;

        begin : zero_check
            reg all_zero;
            all_zero = 1;
            for (i = 0; i < 8; i = i + 1) begin
                read_feature(i[2:0], feat_val);
                if (feat_val != 8'd0) all_zero = 0;
            end
            if (all_zero) begin
                $display("  PASS: All features zero for zero input");
                pass_count = pass_count + 1;
            end else begin
                $display("  FAIL: Some features non-zero for zero input");
                fail_count = fail_count + 1;
            end
        end

        // --- Summary ---
        $display("");
        $display("==========================================");
        $display("  Feature Extraction Testbench Results");
        $display("  PASSED: %0d  FAILED: %0d", pass_count, fail_count);
        $display("==========================================");
        if (fail_count > 0) $display("  *** TEST FAILED ***");
        else                $display("  *** ALL TESTS PASSED ***");

        #100;
        $finish;
    end

    // Timeout watchdog
    initial begin
        #50_000_000;
        $display("WATCHDOG: Simulation timeout");
        $finish;
    end

endmodule
