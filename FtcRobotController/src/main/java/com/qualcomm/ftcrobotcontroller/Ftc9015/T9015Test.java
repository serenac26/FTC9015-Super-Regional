package com.qualcomm.ftcrobotcontroller.Ftc9015;

/**
 * Autonomous program for going up the blue ramp on home side
 *
 * Key point: Robot is programmed to go forward in front of the ramp and backwards to ensure there is no debris blocking the turn path
 *
 *
 * Created by ttn on 12/29/2015.
 */

public class T9015Test extends T9015Hardware {
    private int v_state;
    private boolean v_inState;
    //private boolean v_forward;
    //private boolean v_turn_left;

    private double degrees = 0;
    private double power = 0;

    private long delayStart;

    public T9015Test(){

    }

    @Override
    public void init(){
        super.init();
        v_state = 0;
        v_inState = false;
    }

    @Override
    public void loop(){
        switch(v_state) {
            case 0:
                // Reset the encoders to ensure they are at a known good value.
                telemetry.addData("0 - ","init");
                set_hang_encoders();
                move_to_next_state();
                break;
            case 1:
                //only need to initialize encoders on first time in state
                degrees    = 60;  //set distance to move (ticks)
                power = 0.2;  //set power
                if (first_time_in_state()) {
                    hang_forward();
                }
                telemetry.addData("1 - ","forward=" + degrees + "p=" + power); //displays distance and power to phone screen
                if (has_hang_moved(degrees, power))
                    move_to_next_state();
                break;
            case 2:
                // allow the encoder to reset
                if (have_hang_encoders_reset())
                    move_to_next_state();
                break;
            case 3:
                degrees = 15;  //set distance to move (ticks)
                power = 0.2;  //set power
                if (first_time_in_state()) {
                    set_hang_encoders();
                    hang_backward();
                }
                telemetry.addData("3 - ","backward=" + degrees + "p=" + power); //displays distance and power to phone screen
                if (has_hang_moved(degrees, power))
                    move_to_next_state();
                break;
            case 4:
                // allow the encoder to reset
                if (have_hang_encoders_reset())
                    move_to_next_state();
                break;
            case 5:
                degrees = 45;
                power = 0.25;  //set power
                if (first_time_in_state()) {
                    set_hang_encoders();
                    hang_forward();
                    delayStart = System.currentTimeMillis();    // set timer
                }
                telemetry.addData("5 - ", "forward=" + degrees + "p=" + power); //displays distance and power to phone screen

                // wait till the hanger moved to the desired degree, or time expired.
                // The timer will help the case when the target is reached with a short degree.
                if (has_hang_moved(degrees, power) || System.currentTimeMillis() - delayStart > 2000){
                    // this is the last step of the hanger, a few steps needed to be handled.
                    reset_hang_encoders();  // in case the hanger did not reach the position (blocked?), we help it to reset.
                    // there still some power in the hanger, which provent the hanger from dropping
                    move_to_next_state();
                }

                break;
            case 6:
                if (first_time_in_state())
                    delayStart = System.currentTimeMillis();    // reset timer

                // wait till hang reset, also wait the climers become stable on top of the box
                if (have_hang_encoders_reset() && System.currentTimeMillis() - delayStart > 2000) {
                    move_to_next_state();
                }
                break;
            case 7:
                telemetry.addData("6 - ", "Drop climber");
                drop_climber();
                set_hang_power(0.0f); // cut off the hanger power.
                move_to_next_state();
                break;
            default:
                // The autonomous actions have been accomplished (i.e. the state has
                // transitioned into its final state.
                //
                break;
        }

    }


    //move to next state in switch loop
    void move_to_next_state(){
        v_state++;
        v_inState = false;
    }

    //check if first time in current state
    boolean first_time_in_state(){
        boolean firstTime = v_inState == false;
        v_inState = true;
        return firstTime;
    }

}
