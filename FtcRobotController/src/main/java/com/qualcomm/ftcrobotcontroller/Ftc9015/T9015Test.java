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
    private double distance = 0;
    private double power = 0;

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
                telemetry.addData("0 - ", "test");
                // Reset the encoders to ensure they are at a known good value.
                init_state();
                break;
            case 1:
                //only need to initialize encoders on first time in state
                distance    = 2000;  //set distance to move (ticks)
                power = 0.2;  //set power
                if (first_time_in_state()) {
                    has_hang_forward(distance,power);
                }
                telemetry.addData("4 - ","backward=" + distance + "p=" + power); //displays distance and power to phone screen
                if (has_hang_forward(distance, power))//when encoder has reached corresponding ticks for set distance move to the next state
                    move_to_next_state();
            default:
                set_hang_power(0);
                //
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

    void init_state() {
        telemetry.addData("0 - ","init");
        // Reset the encoders to ensure they are at a known good value.
        reset_hang_encoders();
        move_to_next_state();
    }

}
