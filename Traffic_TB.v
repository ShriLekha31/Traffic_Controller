// =============================================================
// Testbench for Traffic (traffic light + pedestrian controller)
// -------------------------------------------------------------
// - Generates a free-running clock (10 time-unit period).
// - Applies a reset pulse at the start.
// - Pulses NS_button then EW_button to exercise the pedestrian
//   request latches.
// - Uses $monitor (not $display) so every signal change is
//   printed automatically, and dumps a VCD for GTKWave viewing.
// - #delay is used freely here because a testbench is NOT
//   synthesizable hardware -- this is the one place in the
//   project where #delay is appropriate.
// =============================================================

module Traffic_TB;
reg clk,rst,NS_button,EW_button;
wire NS_G, NS_Y, NS_R, EW_G, EW_Y, EW_R; 
wire [2:0] NS_Ped;
wire [2:0] EW_Ped;
wire NS_ped_req, EW_ped_req;

Traffic DUT(.rst(rst), .clk(clk), .NS_G(NS_G), .NS_Y(NS_Y), .NS_R(NS_R), .EW_G(EW_G), .EW_Y(EW_Y), .EW_R(EW_R), 
            .NS_Ped(NS_Ped), .EW_Ped(EW_Ped), .NS_ped_req(NS_ped_req), .EW_ped_req(EW_ped_req), .NS_button(NS_button), .EW_button(EW_button));

// Free-running clock: 10 time-unit period (5 high / 5 low).
always #5 clk=~clk;

// Waveform dump setup, kept in its own initial block.
initial begin
$dumpfile("Traffic.vcd");
$dumpvars(0,Traffic_TB);
end 

// Reset sequencing + simulation end time.
initial begin
clk=0; rst=0;
#5 rst=1;
#10 rst=0;
#1000;
$finish;
end

// Pedestrian button stimulus:
//   t=10  : NS_button pressed
//   t=110 : NS_button released, EW_button pressed
//   t=120 : EW_button released
// Both buttons are explicitly released back to 0 -- if a button
// signal is left high indefinitely, its request latch's
// "button pressed" branch keeps re-triggering every clock cycle
// and will never let the Walk-phase clear condition win, even
// though the design logic itself is correct.
initial begin
#10 NS_button=1; EW_button=0;
#100 NS_button=0; EW_button=1;
#10 EW_button=0;
end

// DUT.u2.state / DUT.u1.timer_done are internal signals reached
// via hierarchical path (through the instance names u1/u2 inside
// the top module DUT) -- useful for debugging even though they
// are not top-level ports.
initial begin
$monitor($time ," NS_G=%b, NS_Y=%b, NS_R=%b, EW_G=%b, EW_Y=%b, EW_R=%b, NS_Ped=%b, EW_Ped=%b, NS_ped_req=%b, EW_ped_req=%b, NS_button=%b, EW_button=%b, state=%b, timer_done=%b", NS_G, NS_Y, NS_R, EW_G, EW_Y, EW_R, NS_Ped, EW_Ped, NS_ped_req, EW_ped_req, NS_button, EW_button, DUT.u2.state, DUT.u1.timer_done);
end

endmodule
