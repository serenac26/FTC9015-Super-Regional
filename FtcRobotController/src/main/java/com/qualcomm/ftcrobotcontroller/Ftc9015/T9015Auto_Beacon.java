package com.qualcomm.ftcrobotcontroller.Ftc9015;

/**
 * Autonomous program for going up the blue ramp on home side
 *
 * Key point: Robot is programmed to go forward in front of the ramp and backwards to ensure there is no debris blocking the turn path
 *
 *
 * Created by seren_000 on 12/29/2015.
 */

public class T9015Auto_Beacon extends T9015Hardware {
    private int v_state;
    private boolean v_inState;
    private double degrees = 0;
    private double distance = 0;
    private double power = 0;

    private double start_time = 0;

    FtcConfig ftcConfig=new FtcConfig();
    boolean redAlliance;

    public T9015Auto_Beacon() {

    }

    @Override
    public void init() {
        super.init();
        v_state = 0;
        v_inState = false;
        //ftcConfig.init(hardwareMap.appContext, this);
        redAlliance = false; //ftcConfig.param.colorIsRed;
        telemetry.addData("0 - ", "init RED=" + redAlliance);
    }

    @Override
    public void loop() {
        switch (v_state) {
            case 0:
                telemetry.addData("s0 - ", "init");
                set_servo_down(); //servo is connected to plate that deflects debris from wheels
                set_hang_encoders();
                hang_forward();
                move_to_next_state();
                break;
            case 1:
                telemetry.addData("s1 - ", "drive to tape");
                drive_to_tape_state(RobotInfo.WHITE_TAPE, 0.2);
                break;
            case 2:
                telemetry.addData("s2 - ", "drive to tape");
                set_drive_power(0.1, 0.1);
                display_ods();

                if (reach_delay_msec(700))
                {
                    set_drive_power(0, 0);
                    move_to_next_state();
                }
                break;

            case 3:
                telemetry.addData("s4 - ", "drive back to tape");
                drive_back_to_tape_state(RobotInfo.WHITE_TAPE, 0.15);
                break;
            case 4:
                // allow the encoder to reset before turning
                if (first_time_in_state())
                    reset_drive_encoders();
                if (have_drive_encoders_reset())
                    move_to_next_state();
                break;
            case 5:
                telemetry.addData("s3 - ","turn");
                if (redAlliance) {
                    turn_to_state(RobotInfo.TURN_LEFT, 45, 0.3);
                }
                else
                {
                    turn_to_state(RobotInfo.TURN_RIGHT, 45, 0.3);
                }
                break;
            case 6:
                // reset motor not to use encoder anymore
                run_without_drive_encoders();
                move_to_next_state();
                break;
            case 7:
                telemetry.addData("s5 - ", "line follow");
                drive_forward_state(0.07);
                break;
            case 8:
                hang_forward_state(80, 0.3);
                break;
            case 9:
                telemetry.addData("s10 - ", "drop climber");
                if (reach_delay_msec(2000))
                {
                    set_drive_power(0, 0);
                    drop_climber();
                    move_to_next_state();
                }
                break;
            default:
                set_drive_power(0,0);
                // The autonomous actions have been accomplished (i.e. the state has
                // transitioned into its final state.
                //
                break;
        }

    }

    //move to next state in switch loop
    void move_to_next_state() {
        v_state++;
        v_inState = false;
    }

    //check if first time in current state
    boolean first_time_in_state() {
        boolean firstTime = (v_inState == false);
        v_inState = true;
        return firstTime;
    }

    void init_state() {
        telemetry.addData("0 - ","init");
        // Reset the encoders to ensure they are at a known good value.
        reset_drive_encoders();
        set_servo_down(); //servo is connected to plate that deflects debris from wheels
        move_to_next_state();
    }

    // Delay msec - return false if not timeout yet
    private boolean start_timer = false;
    boolean done = false;
    boolean reach_delay_msec(int msec) {
        double cur_time = System.currentTimeMillis();
        int diff ;
        if (start_timer == false)
        {
            done = false;
            start_timer = true;
            start_time = System.currentTimeMillis();
        }
        diff = (int)(cur_time - start_time);
        if (diff >= msec)
        {
            done = true;
            start_timer = false;
        }
        telemetry.addData("time-" , diff + " "+ msec + " " + cur_time + " " + start_time);
        return done;
    }

    void set_encoder_forward()
    {
        if (first_time_in_state()) {
            run_using_encoders();
            go_forward(); //set direction forward
        }
    }

    void hang_forward_state(double degree, double power)
    {
        telemetry.addData(v_state + " - ", "hang forward=" + degree + "p=" + power); //displays distance and power to phone screen
        // when encoder has reached corresponding ticks for set distance move to the next stat
        if (has_hang_moved(degree, power)) //when encoder has reached corresponding ticks for set distance move to the next state
            move_to_next_state();

    }

    void drive_forward_state(double power)
    {
        telemetry.addData(v_state + " - ", "drive forward" );
        display_ods();
        display_ultrasonic();
        if (first_time_in_state())
        {
            reach_wall_count = 0;
            consecutive_wall_count = 0;
            dead_wall = false;
        }

        drive_forward_no_encoder(power,power);

        telemetry.addData(v_state + " - ", "forward=" + distance + "p=" + power); //displays distance and power to phone screen
        telemetry.addData("line" + " - ", "wall-cnt =" + reach_wall_count + " " + consecutive_wall_count);

        if ((dead_wall == true)||(reach_delay_msec(2500)==true))
        {
            set_drive_power(0, 0);
            move_to_next_state();
        }

    }

    void set_encoder_turn(int direction)
    {
        if (first_time_in_state()) {
            run_using_encoders();
            if (direction==RobotInfo.TURN_LEFT)
                turn_left();
            if (direction==RobotInfo.TURN_RIGHT)
                turn_right(); //set direction to turn left
        }
    }

    void turn_to_state(int direction, double degrees, double power)
    {
        telemetry.addData(v_state + " - ", "turn="+direction+" deg=" + degrees + "p=" + power); //displays distance and power to phone screen
        set_encoder_turn(direction);
        display_ods();
        if (has_driver_turned_degrees(degrees, power)) //when encoder has reached given number of ticks corresponding to given degree measure move to next state
            move_to_next_state();
    }

    void drive_to_tape_state(int tape_color, double power)
    {
        telemetry.addData(v_state + " - ", "drive_to_tape =" + tape_color);
        drive_forward_no_encoder(power, power);
        display_ods();

        // when encoder has reached corresponding ticks for set distance move to the next stat
        if (find_tape_color(tape_color)) //when encoder has reached corresponding ticks for set distance move to the next state
        {
            set_drive_power(0, 0);
            move_to_next_state();
        }
    }

    void drive_back_to_tape_state(int tape_color, double power) {
        telemetry.addData(v_state + " - ", "drive_to_tape =" + tape_color);
        drive_back_no_encoder(power, power);
        display_ods();

        // when encoder has reached corresponding ticks for set distance move to the next stat
        if (find_tape_color(tape_color)) //when encoder has reached corresponding ticks for set distance move to the next state
        {
            set_drive_power(0, 0);
            move_to_next_state();
        }
    }

    int reach_wall_count = 0;
    int consecutive_wall_count=0;
    boolean dead_wall=false;
    boolean previous_wall = false;
    void line_follow_state(int tape_color, double power)
    {
        telemetry.addData(v_state + " - ", "line_follow =" + tape_color);
        display_ods();
        display_ultrasonic();
        if (first_time_in_state())
        {
            reach_wall_count=0;
            consecutive_wall_count = 0;
            dead_wall = false;
        }

        drive_forward_no_encoder(power,power);

      if (find_tape_color(tape_color))
        {
            drive_forward_no_encoder(power,0);
        }
        else
        {
            drive_forward_no_encoder(0,power);
        }

        if (reach_wall(RobotInfo.WALL_CM)) {
            reach_wall_count++;
            if ((reach_wall_count-consecutive_wall_count)==1) {
                consecutive_wall_count++;
            }
            if (consecutive_wall_count>=4)
            {
                dead_wall=true;
            }
        }
        else
        {
            consecutive_wall_count=0;
            if ((reach_wall_count-consecutive_wall_count)>2) {
                reach_wall_count=0;
            }
        }
 
        telemetry.addData("line" + " - ", "wall-cnt =" + reach_wall_count + " " + consecutive_wall_count);

        if ((dead_wall == true)||(reach_delay_msec(2500)==true))
        {
            telemetry.addData(v_state + " - ", "line_follow =" + tape_color + " deadwall");
                set_drive_power(0, 0);
                move_to_next_state();
        }

     }

}
