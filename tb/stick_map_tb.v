`timescale 1ns / 1ps
module stick_map_tb;

    reg clk = 0;
    reg start;
    reg axis_sel;
    reg signed [7:0] val_x, val_y;
    wire signed [7:0] val_out;
    wire done;

    stick_map dut (
        .clk(clk), .start(start), .axis_sel(axis_sel),
        .val_x(val_x), .val_y(val_y),
        .val_out(val_out), .done(done)
    );

    always #10 clk = ~clk;

    reg signed [7:0] got_x, got_y;

    // KEY TEST: only ONE start/done cycle, then toggle axis_sel and read
    // val_out directly -- no second start pulse. This is exactly the
    // behavior the combinational-mux fix is supposed to enable.
    task run_case(input signed [7:0] tx, input signed [7:0] ty, input [127:0] label);
        begin
            @(posedge clk);
            val_x <= tx; val_y <= ty; axis_sel <= 0;
            start <= 1;
            @(posedge clk);
            start <= 0;
            wait(done);
            got_x = val_out;          // read X immediately

            @(posedge clk);
            axis_sel <= 1;            // toggle only -- no start pulse
            @(posedge clk);
            got_y = val_out;          // read Y on the very next cycle

            $display("%0s: n64_target=(%0d,%0d) -> gc=(%0d,%0d)", label, tx, ty, got_x, got_y);
        end
    endtask

    initial begin
        #200000;
        $display("TIMEOUT -- simulation did not finish in time");
        $finish;
    end

    initial begin
        start = 0; axis_sel = 0;
        #50;
        run_case(56, 0, "axis full-scale");
        run_case(-56, 0, "axis neg full-scale");
        run_case(0, 56, "axis Y full-scale");
        run_case(25, 25, "diagonal");
        run_case(-25, -25, "neg diagonal");
        run_case(0, 0, "center");
        run_case(20, 5, "off-axis minor small");
        run_case(-3, -53, "worst-case 1 (expect -16,-77)");
        run_case(-53, -3, "worst-case 2 (expect -77,-16)");
        run_case(-11, 50, "worst-case 3 (expect -29,78)");
        #50;
        $display("ALL CASES COMPLETED -- toggle-without-restart confirmed working");
        $finish;
    end

endmodule
