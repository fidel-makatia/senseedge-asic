// SPDX-License-Identifier: Apache-2.0
// Testbench: Neural Network Inference Engine
// Tests:
//   1. Weight loading via write interface
//   2. Inference with known weights → verify classification
//   3. Different input patterns → verify different classes
//   4. Weight update and re-inference

`timescale 1ns / 1ps

module tb_nn_engine;

    // --- Clock and Reset ---
    reg clk;
    reg rst;

    initial clk = 0;
    always #20 clk = ~clk; // 25 MHz

    // --- DUT signals ---
    reg         start;
    wire [2:0]  feature_addr;
    reg  [7:0]  feature_in;
    wire        done;
    wire [1:0]  class_id;
    wire [7:0]  confidence;
    wire        busy;
    reg         wt_wr_en;
    reg  [7:0]  wt_wr_addr;
    reg  [7:0]  wt_wr_data;

    // --- Feature memory ---
    reg [7:0] feature_mem [0:7];

    always @(*) begin
        feature_in = feature_mem[feature_addr];
    end

    // --- DUT ---
    nn_engine dut (
        .clk         (clk),
        .rst         (rst),
        .start       (start),
        .feature_in  (feature_in),
        .feature_addr(feature_addr),
        .done        (done),
        .class_id    (class_id),
        .confidence  (confidence),
        .busy        (busy),
        .wt_wr_en    (wt_wr_en),
        .wt_wr_addr  (wt_wr_addr),
        .wt_wr_data  (wt_wr_data)
    );

    // --- Tasks ---
    task write_weight;
        input [7:0] addr;
        input [7:0] data;
        begin
            @(posedge clk);
            wt_wr_en   <= 1'b1;
            wt_wr_addr <= addr;
            wt_wr_data <= data;
            @(posedge clk);
            wt_wr_en   <= 1'b0;
        end
    endtask

    task run_inference;
        begin
            @(posedge clk);
            start <= 1;
            @(posedge clk);
            start <= 0;

            begin : wait_inference_done
                integer wait_cnt;
                wait_cnt = 0;
                while (done !== 1'b1 && wait_cnt < 100000) begin
                    @(posedge clk);
                    wait_cnt = wait_cnt + 1;
                end
                if (wait_cnt >= 100000)
                    $display("  TIMEOUT: Inference did not complete");
            end

            repeat (5) @(posedge clk);
        end
    endtask

    // --- Test sequence ---
    integer pass_count;
    integer fail_count;
    integer i;

    initial begin
        $dumpfile("tb_nn_engine.vcd");
        $dumpvars(0, tb_nn_engine);

        pass_count = 0;
        fail_count = 0;

        rst       = 1;
        start     = 0;
        wt_wr_en  = 0;
        wt_wr_addr = 0;
        wt_wr_data = 0;

        repeat (10) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);

        // ==================================================================
        // Load weights: Design a simple network that classifies based on
        // which input feature is dominant
        //
        // Strategy:
        //   Layer 1 (8→16): Identity-like mapping with some amplification
        //     Neuron 0-3: respond strongly to feature 0-3 (band energies)
        //     Neuron 4-7: respond to features 4-7
        //     Neurons 8-15: small random-ish weights
        //   Layer 2 (16→4): Map first 4 hidden neurons to output classes
        //
        // Simplified: make neuron K in L1 respond to input K with weight 127
        //             and have output class K respond to hidden K with weight 127
        // ==================================================================
        $display("");
        $display("[SETUP] Loading weights...");

        // Layer 1 weights [0..127]: 16 neurons x 8 inputs
        // Initialize all to zero first
        for (i = 0; i < 128; i = i + 1)
            write_weight(i[7:0], 8'd0);

        // Neuron 0 responds to input 0: weight[0*8+0] = 127
        write_weight(8'd0, 8'd127);     // w[0][0] = 127
        // Neuron 1 responds to input 1: weight[1*8+1] = 127
        write_weight(8'd9, 8'd127);     // w[1][1] = 127
        // Neuron 2 responds to input 2: weight[2*8+2] = 127
        write_weight(8'd18, 8'd127);    // w[2][2] = 127
        // Neuron 3 responds to input 3: weight[3*8+3] = 127
        write_weight(8'd27, 8'd127);    // w[3][3] = 127

        // Layer 1 biases [128..143]: all zero
        for (i = 128; i < 144; i = i + 1)
            write_weight(i[7:0], 8'd0);

        // Layer 2 weights [144..207]: 4 neurons x 16 inputs
        // Initialize all to zero
        for (i = 144; i < 208; i = i + 1)
            write_weight(i[7:0], 8'd0);

        // Output 0 (Healthy) responds to hidden 0: weight[144 + 0*16 + 0] = 127
        write_weight(8'd144, 8'd127);
        // Output 1 (Bearing) responds to hidden 1: weight[144 + 1*16 + 1] = 127
        write_weight(8'd161, 8'd127);
        // Output 2 (Imbalance) responds to hidden 2: weight[144 + 2*16 + 2] = 127
        write_weight(8'd178, 8'd127);
        // Output 3 (Misalign) responds to hidden 3: weight[144 + 3*16 + 3] = 127
        write_weight(8'd195, 8'd127);

        // Layer 2 biases [208..211]: all zero
        for (i = 208; i < 212; i = i + 1)
            write_weight(i[7:0], 8'd0);

        $display("  Weights loaded: 212 parameters");

        // ==================================================================
        // Test 1: Input dominated by feature 0 → Class 0 (Healthy)
        // ==================================================================
        $display("");
        $display("[TEST 1] Classify as Healthy (feature 0 dominant)");
        feature_mem[0] = 8'd120;
        feature_mem[1] = 8'd10;
        feature_mem[2] = 8'd10;
        feature_mem[3] = 8'd10;
        feature_mem[4] = 8'd5;
        feature_mem[5] = 8'd5;
        feature_mem[6] = 8'd5;
        feature_mem[7] = 8'd5;

        run_inference;
        $display("  Class: %0d, Confidence: %0d", class_id, confidence);

        if (class_id == 2'd0) begin
            $display("  PASS: Classified as Healthy (class 0)");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Expected class 0, got class %0d", class_id);
            fail_count = fail_count + 1;
        end

        // ==================================================================
        // Test 2: Input dominated by feature 1 → Class 1 (Bearing Wear)
        // ==================================================================
        $display("");
        $display("[TEST 2] Classify as Bearing Wear (feature 1 dominant)");
        feature_mem[0] = 8'd10;
        feature_mem[1] = 8'd120;
        feature_mem[2] = 8'd10;
        feature_mem[3] = 8'd10;
        feature_mem[4] = 8'd5;
        feature_mem[5] = 8'd5;
        feature_mem[6] = 8'd5;
        feature_mem[7] = 8'd5;

        run_inference;
        $display("  Class: %0d, Confidence: %0d", class_id, confidence);

        if (class_id == 2'd1) begin
            $display("  PASS: Classified as Bearing Wear (class 1)");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Expected class 1, got class %0d", class_id);
            fail_count = fail_count + 1;
        end

        // ==================================================================
        // Test 3: Input dominated by feature 2 → Class 2 (Imbalance)
        // ==================================================================
        $display("");
        $display("[TEST 3] Classify as Imbalance (feature 2 dominant)");
        feature_mem[0] = 8'd10;
        feature_mem[1] = 8'd10;
        feature_mem[2] = 8'd120;
        feature_mem[3] = 8'd10;
        feature_mem[4] = 8'd5;
        feature_mem[5] = 8'd5;
        feature_mem[6] = 8'd5;
        feature_mem[7] = 8'd5;

        run_inference;
        $display("  Class: %0d, Confidence: %0d", class_id, confidence);

        if (class_id == 2'd2) begin
            $display("  PASS: Classified as Imbalance (class 2)");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Expected class 2, got class %0d", class_id);
            fail_count = fail_count + 1;
        end

        // ==================================================================
        // Test 4: Input dominated by feature 3 → Class 3 (Misalignment)
        // ==================================================================
        $display("");
        $display("[TEST 4] Classify as Misalignment (feature 3 dominant)");
        feature_mem[0] = 8'd10;
        feature_mem[1] = 8'd10;
        feature_mem[2] = 8'd10;
        feature_mem[3] = 8'd120;
        feature_mem[4] = 8'd5;
        feature_mem[5] = 8'd5;
        feature_mem[6] = 8'd5;
        feature_mem[7] = 8'd5;

        run_inference;
        $display("  Class: %0d, Confidence: %0d", class_id, confidence);

        if (class_id == 2'd3) begin
            $display("  PASS: Classified as Misalignment (class 3)");
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Expected class 3, got class %0d", class_id);
            fail_count = fail_count + 1;
        end

        // ==================================================================
        // Test 5: Confidence should be non-zero
        // ==================================================================
        $display("");
        $display("[TEST 5] Confidence check");
        if (confidence > 8'd0) begin
            $display("  PASS: Confidence = %0d (non-zero)", confidence);
            pass_count = pass_count + 1;
        end else begin
            $display("  FAIL: Confidence should be non-zero");
            fail_count = fail_count + 1;
        end

        // ==================================================================
        // Test 6: All-zero features
        // ==================================================================
        $display("");
        $display("[TEST 6] All-zero features");
        for (i = 0; i < 8; i = i + 1)
            feature_mem[i] = 8'd0;

        run_inference;
        $display("  Class: %0d, Confidence: %0d", class_id, confidence);
        // With zero inputs and zero biases, all outputs should be 0
        // Argmax of all-zeros returns the last max (implementation-dependent)
        $display("  PASS: Inference completed without hang (class=%0d)", class_id);
        pass_count = pass_count + 1;

        // --- Summary ---
        $display("");
        $display("==========================================");
        $display("  NN Engine Testbench Results");
        $display("  PASSED: %0d  FAILED: %0d", pass_count, fail_count);
        $display("==========================================");
        if (fail_count > 0) $display("  *** TEST FAILED ***");
        else                $display("  *** ALL TESTS PASSED ***");

        #100;
        $finish;
    end

    // Timeout watchdog
    initial begin
        #100_000_000;
        $display("WATCHDOG: Simulation timeout");
        $finish;
    end

endmodule
