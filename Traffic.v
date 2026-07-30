// =============================================================
// Traffic Light Controller with Pedestrian Signals
// -------------------------------------------------------------
// Design style: Control Path / Datapath split
//   - Traffic_Data    : datapath  (parameterized down-counter,
//                        generates timer_done)
//   - Traffic_Control : control path (FSM, Ld_G/Ld_Y pulse
//                        generation, car lights, pedestrian
//                        Walk/Flash/Don't-Walk outputs,
//                        pedestrian request latches)
//   - Traffic         : top-level module wiring the two
//                        together
// =============================================================


// ---------------------------------------------------------------
// Data Path: parameterized counter used to time each FSM state.
// Counts up to (Count_G-1) or (Count_Y-1) depending on which load
// signal (Ld_G / Ld_Y) is pulsed by the control path, then asserts
// timer_done for exactly one clock cycle.
// ---------------------------------------------------------------
module Traffic_Data(Ld_G, Ld_Y, timer_done, clk, rst);
   input Ld_G, Ld_Y, clk, rst;
   output reg timer_done;
   reg [5:0] count;                    // current count value
   parameter Count_G=20, Count_Y=10;    // duration (in clk ticks) for Green / Yellow
   reg [5:0] target;                   // holds whichever duration is currently active

always@(posedge clk)
begin
if( rst )                               // Reset: force count/target/timer_done to known values. 
    begin
    count <=1'b0;
    timer_done <= 1'b0;
    target <= 1'b0;
    end
else if ( Ld_G || Ld_Y)
    begin
    // A new phase has just started (Ld_G/Ld_Y is a ONE-CYCLE PULSE
    // from Traffic_Control, not held high). Latch the correct
    // target duration and restart the count from 0 in the SAME
    // cycle, using two independent non-blocking assignments.
    target <= (Ld_G ? Count_G : Count_Y );
    count <= 1'b0;
    end
else if ( count == target-1)
    begin
    // Reached the target duration -> pulse timer_done for
    // exactly one cycle and roll count back to 0.
    count <= 1'b0;
    timer_done <= 1'b1;
    end
else
    begin
    // Normal counting cycle.
    count <= count+1;
    timer_done <= 1'b0;
    end
end
endmodule


// ---------------------------------------------------------------
// Control Path: 4-state Moore FSM (S0..S3) driving the car lights,
// pedestrian Walk/Flash/Don't-Walk signals, and pedestrian
// "request registered" latches.
//
// State map:
//   S0 : NS=Green , EW=Red
//   S1 : NS=Yellow, EW=Red
//   S2 : NS=Red   , EW=Green
//   S3 : NS=Red   , EW=Yellow
// ---------------------------------------------------------------
 module Traffic_Control(timer_done,Ld_G,Ld_Y,rst,clk,NS_Y, NS_R,NS_G, EW_G, EW_Y, EW_R, NS_Ped, EW_Ped, NS_ped_req, EW_ped_req, NS_button, EW_button);
    input timer_done,clk,rst;
    input NS_button, EW_button;         // momentary pedestrian pushbuttons
    output  Ld_G,Ld_Y;                  // one-cycle load pulses sent to Traffic_Data
    reg [1:0] state,prv_state;          // prv_state is used only to detect state ENTRY (edge)
    parameter s0=2'b00, s1=2'b01, s2=2'b10, s3=2'b11;
    output reg  NS_Y, NS_R, NS_G, EW_G, EW_Y, EW_R;
    output reg [2:0] NS_Ped, EW_Ped;    // one-hot: bit2=Don't Walk, bit1=Flashing, bit0=Walk
    output reg NS_ped_req, EW_ped_req;  // debug/LED outputs: pedestrian request latched?

// -----------------------------------------------------------
// State register + next-state logic.
// Priority: reset > normal state transition.
// prv_state is updated every non-reset cycle so that
// Ld_G/Ld_Y (below) can detect "just entered this state"
// rather than "still sitting in this state".
// -----------------------------------------------------------
always@(posedge clk)
    begin
       if ( rst )
       begin
       state <= s0;
       // NOTE: prv_state is deliberately primed to s3 (not s0) on
       // reset. If it were primed to s0, then on the first cycle
       // after reset state==s0 AND prv_state==s0, so the Ld_G
       // "just entered s0" pulse below would NEVER fire and the
       // datapath counter would never get its first load.
       prv_state <= s3;
       end
    else 
    begin
    case(state)
    s0 : begin
         state <= timer_done ? s1 : s0;
         end
    s1 : begin
         state <= timer_done ? s2 : s1;
         end
    s2 : begin
         state <= timer_done ? s3 : s2;
         end
    s3 : begin
         state <= timer_done ? s0 : s3;
         end
    default : state <= s0;
    endcase
    prv_state <= state;   // capture OLD value of state (non-blocking, so this
                           // is safe regardless of ordering vs. the case above)
    end
    end

// -----------------------------------------------------------
// Ld_G / Ld_Y: one-cycle pulses, high only on the exact clock
// edge where the FSM just transitioned INTO s0/s2 (Ld_G) or
// s1/s3 (Ld_Y). Using (state==sX && prv_state!=sX) instead of
// just (state==sX) prevents the datapath from being reloaded
// every single cycle, which would otherwise hold count at 0
// forever and the FSM would never advance ("perpetual reload"
// bug).
// -----------------------------------------------------------
assign Ld_G = (state == s0 && prv_state != s0 ) || (state == s2 && prv_state != s2 ) ;
assign Ld_Y = (state == s1 && prv_state != s1 ) || (state == s3 && prv_state != s3 ) ;

// -----------------------------------------------------------
// Pedestrian request latches (set-reset behaviour).
// Priority order: reset > button press (set) > Walk-phase-start (clear).
// Reset is always checked first so a system reset can never be
// overridden by a button press.
//
// NS latch: pedestrians crossing NS-flowing traffic get Walk
// during S2/S3 (NS lights are Red there) -> that is the clear
// condition. EW latch mirrors this for S0/S1.
// No explicit "hold" branch is needed: a reg not assigned in a
// given clock edge automatically keeps its previous value.
// -----------------------------------------------------------
always@(posedge clk)
    begin
    if ( rst )
         NS_ped_req <= 1'b0;
    else if ( NS_button )
             NS_ped_req <= 1'b1;
    else if ( state == s2 || state ==s3 )
             NS_ped_req <= 1'b0;
    end

always@(posedge clk)
    begin
    if ( rst )
         EW_ped_req <= 1'b0;
    else if ( EW_button )
             EW_ped_req <= 1'b1;
    else if ( state == s0 || state ==s1 )
             EW_ped_req <= 1'b0;
    end

// -----------------------------------------------------------
// Combinational output logic (Moore-style: outputs depend only
// on `state`, never on timer_done or the buttons directly).
// Every branch, including default, assigns ALL outputs to avoid
// unintended latches. `default` falls back to all-Red / all
// Don't-Walk as the safe condition for an unreachable state.
// -----------------------------------------------------------
always@(*)
    begin
    case (state)
    s0 : begin
         NS_G = 1'b1; EW_R = 1'b1;
         NS_Ped = 3'b001; EW_Ped = 3'b100;   // NS Don't-Walk, EW Walk
         NS_Y = 1'b0; NS_R = 1'b0; EW_G = 1'b0; EW_Y = 1'b0;
         end
    s1 : begin
         NS_Y = 1'b1; EW_R = 1'b1;
         NS_Ped = 3'b010; EW_Ped = 3'b100;   // NS Flashing, EW Walk
         NS_G = 1'b0; NS_R = 1'b0; EW_G = 1'b0; EW_Y = 1'b0;
         end
    s2 : begin
         NS_R = 1'b1; EW_Y = 1'b1;
         NS_Ped = 3'b100; EW_Ped = 3'b010;   // NS Walk, EW Flashing
         NS_Y = 1'b0; NS_G = 1'b0; EW_G = 1'b0; EW_R = 1'b0;
         end
    s3 : begin
         NS_R = 1'b1; EW_G = 1'b1;
         NS_Ped = 3'b100; EW_Ped = 3'b001;   // NS Walk, EW Don't-Walk
         NS_Y = 1'b0; NS_G = 1'b0; EW_R = 1'b0; EW_Y = 1'b0;
         end
    default : begin
              // Safe fallback: both directions Red, both pedestrian
              // signals Don't-Walk.
              NS_R = 1'b1; EW_R = 1'b1;
              NS_Ped = 3'b001; EW_Ped = 3'b001;
              NS_G = 1'b0; NS_Y = 1'b0; EW_G = 1'b0; EW_Y = 1'b0;
              end
    endcase
    end


endmodule   

// ---------------------------------------------------------------
// Top-level module: instantiates the datapath and control-path
// modules and wires them together. Ld_G, Ld_Y, and timer_done are
// purely internal signals (control<->datapath handshake) and are
// therefore declared as internal wires, not top-level ports.
// ---------------------------------------------------------------
module Traffic(rst, clk, NS_G, NS_Y, NS_R, EW_G, EW_Y, EW_R, NS_Ped, EW_Ped, NS_ped_req, EW_ped_req, NS_button, EW_button);
    input clk, rst, NS_button, EW_button;
    output NS_G, NS_Y, NS_R, EW_G, EW_Y, EW_R; 
    output [2:0] NS_Ped;
    output [2:0] EW_Ped;
    wire Ld_G, Ld_Y, timer_done;        // internal control<->datapath handshake signals
    output NS_ped_req, EW_ped_req;      // pedestrian-request debug/LED outputs

Traffic_Data u1(.Ld_G(Ld_G), .Ld_Y(Ld_Y), .clk(clk), .rst(rst), .timer_done(timer_done));
Traffic_Control u2(.timer_done(timer_done), .rst(rst), .clk(clk), .Ld_G(Ld_G), .Ld_Y(Ld_Y), 
                   .NS_G(NS_G), .NS_Y(NS_Y), .NS_R(NS_R), .EW_G(EW_G), .EW_Y(EW_Y), .EW_R(EW_R),
                   .NS_Ped(NS_Ped), .EW_Ped(EW_Ped), .NS_ped_req(NS_ped_req), .EW_ped_req(EW_ped_req), .NS_button(NS_button), .EW_button(EW_button));

endmodule
