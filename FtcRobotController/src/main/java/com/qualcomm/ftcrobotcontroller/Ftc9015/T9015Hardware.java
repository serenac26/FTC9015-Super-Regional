package com.qualcomm.ftcrobotcontroller.Ftc9015;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

//------------------------------------------------------------------------------
//
// PushBotHardware
//

/**
 * Provides a single hardware access point between custom op-modes and the
 * OpMode class for the Push Bot.
 *
 * This class prevents the custom op-mode from throwing an exception at runtime.
 * If any hardware fails to map, a warning will be shown via telemetry data,
 * calls to methods will fail, but will not cause the application to crash.
 *
 * @author SSI Robotics
 * @version 2015-08-13-20-04
 */
public class T9015Hardware extends OpMode

{
    //--------------------------------------------------------------------------
    //
    // PushBotHardware
    //
    /**
     * Construct the class.
     *
     * The system calls this member when the class is instantiated.
     */
    final static double H_ARM_MIN_RANGE  = 0.52;
    final static double H_ARM_MAX_RANGE  = 0.95;

    final static double H_BACK_MIN_RANGE  = 0.001;
    final static double H_BACK_MAX_RANGE  = .999;

    final static double CLIMBER_START  = 0.999;
    final static double CLIMBER_END    = 0.001;

    private final int TICKS_PER_REV = 1120;
    private final double WHEEL_DIAMETER = 10.16; //in centimeters
    private final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    private final int TURN_45_DEGREES = 1000;

    final int HANGER_45_DEGREES = 100; // # of ticks per 45 degree.



    public T9015Hardware()

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.

    } //

    //--------------------------------------------------------------------------
    //
    // init
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    @Override public void init ()

    {
        //
        // Use the hardwareMap to associate class members to hardware ports.
        //
        // Note that the names of the devices (i.e. arguments to the get method)
        // must match the names specified in the configuration file created by
        // the FTC Robot Controller (Settings-->Configure Robot).
        //
        // The variable below is used to provide telemetry data to a class user.
        //
        v_warning_generated = false;
        v_warning_message = "Can't map; ";

        //
        // Connect the drive wheel motors.
        //
        // The direction of the right motor is reversed, so joystick inputs can
        // be more generically applied.
        //
        try
        {
            v_motor_f_left_drive = hardwareMap.dcMotor.get ("motor1");
        }
        catch (Exception p_exception)
        {
            m_warning_message ("motor1");
            DbgLog.msg(p_exception.getLocalizedMessage());

            v_motor_f_left_drive = null;
        }

        try
        {
            v_motor_f_right_drive = hardwareMap.dcMotor.get ("motor2");
        }
        catch (Exception p_exception)
        {
            m_warning_message ("motor2");
            DbgLog.msg (p_exception.getLocalizedMessage ());

            v_motor_f_right_drive = null;
        }

        set_direction_forward(true);

        try
        {
            v_motor_r_left_drive = hardwareMap.dcMotor.get ("motor3");
        }
        catch (Exception p_exception)
        {
            m_warning_message ("motor3");
            DbgLog.msg (p_exception.getLocalizedMessage ());

            v_motor_r_left_drive = null;
        }

        try
        {
            v_motor_r_right_drive = hardwareMap.dcMotor.get ("motor4");
        }
        catch (Exception p_exception) {
            m_warning_message("motor4");
            DbgLog.msg(p_exception.getLocalizedMessage());

            v_motor_r_right_drive = null;
        }

        try
        {
            v_motor_puller = hardwareMap.dcMotor.get ("pull");
        }
        catch (Exception p_exception)
        {
            m_warning_message ("pull");
            DbgLog.msg (p_exception.getLocalizedMessage ());

            v_motor_puller = null;
        }

        try
        {
            v_motor_slider = hardwareMap.dcMotor.get ("slide");
        }
        catch (Exception p_exception)
        {
            m_warning_message ("pull");
            DbgLog.msg (p_exception.getLocalizedMessage ());

            v_motor_slider = null;
        }

        try
        {
            v_motor_left_hang = hardwareMap.dcMotor.get ("hang1");
        }
        catch (Exception p_exception)
        {
            m_warning_message ("hang1");
            DbgLog.msg (p_exception.getLocalizedMessage ());

            v_motor_left_hang = null;
        }
        try
        {
            v_motor_right_hang = hardwareMap.dcMotor.get ("hang2");
        }
        catch (Exception p_exception)
        {
            m_warning_message ("hang2");
            DbgLog.msg (p_exception.getLocalizedMessage ());

            v_motor_right_hang = null;
        }
        //
        // Connect the servo motors.
        //
        // Indicate the initial position of both the left and right servos.  The
        // hand should be halfway opened/closed.
        //
        try
        {
            v_servo_puller = hardwareMap.servo.get ("spull");
        }
        catch (Exception p_exception)
        {
            m_warning_message ("spull");
            DbgLog.msg (p_exception.getLocalizedMessage ());

            v_servo_puller = null;
        }

        try
        {
            v_servo_back = hardwareMap.servo.get ("sback");
        }
        catch (Exception p_exception)
        {
            m_warning_message ("sback");
            DbgLog.msg (p_exception.getLocalizedMessage ());

            v_servo_back = null;
        }

        try
        {
            v_servo_climber = hardwareMap.servo.get ("sclimber");
        }
        catch (Exception p_exception)
        {
            m_warning_message ("sclimber");
            DbgLog.msg (p_exception.getLocalizedMessage ());

            v_servo_climber = null;
        }

        try
        {
            v_servo_beacon = hardwareMap.servo.get ("sbeacon");
        }
        catch (Exception p_exception)
        {
            m_warning_message ("sbeacon");
            DbgLog.msg (p_exception.getLocalizedMessage ());

            v_servo_beacon = null;
        }

        double l_hand_position = 0.5;

        set_direction_forward(true);

        try
        {
            v_ods = hardwareMap.opticalDistanceSensor.get ("ods");
                if (v_ods != null) {
                    v_ods.enableLed(true);
               }
        }
        catch (Exception p_exception)
        {
            m_warning_message ("ods");
            DbgLog.msg (p_exception.getLocalizedMessage ());

            v_ods = null;
        }

        try
        {
            v_ultra = hardwareMap.ultrasonicSensor.get ("ultra");
        }
        catch (Exception p_exception)
        {
            m_warning_message ("ultra");
            DbgLog.msg (p_exception.getLocalizedMessage ());

            v_ultra = null;
        }

        try
        {
            v_touch = hardwareMap.touchSensor.get ("touch");
        }
        catch (Exception p_exception)
        {
            m_warning_message ("touch");
            DbgLog.msg (p_exception.getLocalizedMessage ());

            v_touch = null;
        }

        try
        {
            v_color = hardwareMap.colorSensor.get ("mr_color");
            if (v_color!=null)
            {
                v_color.enableLed(false); // passive mode
            }
        }
        catch (Exception p_exception)
        {
            m_warning_message ("mr_color");
            DbgLog.msg (p_exception.getLocalizedMessage ());

            v_color = null;
        }

    } // init

    void beacon_left()
    {
        v_servo_beacon.setPosition(RobotInfo.BEACON_LEFT);
    }

    void beacon_right()
    {
        v_servo_beacon.setPosition(RobotInfo.BEACON_RIGHT);
    }

    void beacon_down()
    {
        v_servo_beacon.setPosition(RobotInfo.BEACON_DOWN);
    }

    double us_value=0;
    boolean reach_wall(double distance)
    {
        boolean find_wall = false;
        if (v_ultra !=null) {
        while (us_value==0) {
           us_value = v_ultra.getUltrasonicLevel();
        }
        telemetry.addData("reach_wall-", "ultra=" + us_value + " " + distance + " " + find_wall);
        if (us_value <= distance) find_wall = true;
        }
        return find_wall;
    }

    boolean find_tape_color(int tape_color)
    {
        double ods_val =0;
        int    ods_int =0;
        boolean find_tape = false;
        if (v_ods !=null) {
            ods_val = v_ods.getLightDetected();
            ods_int = v_ods.getLightDetectedRaw();
        }
        if (ods_int >= tape_color) find_tape = true;
        telemetry.addData("ods-", "ods=" + ods_int + " " + ods_val + " tape_color=" + tape_color + " " + find_tape);
        return find_tape;
    }

    void display_color()
    {
        if (v_color != null) {
            telemetry.addData("Clear", v_color.alpha());
            telemetry.addData("Red  ", v_color.red());
            telemetry.addData("Green", v_color.green());
            telemetry.addData("Blue ", v_color.blue());
        }
        else
        {
            telemetry.addData(" ERR -", "NO color sensor ");
        }
    }

    void display_ods()
    {
        double ods_val =0;
        int    ods_int =0;
        if (v_ods !=null) {
            ods_val = v_ods.getLightDetected();
            ods_int = v_ods.getLightDetectedRaw();
        }
        telemetry.addData("ods-", "ods=" + ods_int + " " + ods_val + " tape_color=" );
    }

    void display_ultrasonic()
    {
        if (v_ultra !=null) {
            us_value = v_ultra.getUltrasonicLevel();
        }
        telemetry.addData("ultra-", "ultra=" + us_value );
    }

    void reset_climber ()
    {
        if (v_servo_climber != null)
        {
            v_servo_climber.setPosition(RobotInfo.CLIMBER_LOCK_POSITION);
        }

    }
    void drop_climber ()
    {
        if (v_servo_climber != null)
        {
            v_servo_climber.setPosition(RobotInfo.CLIMBER_RELEASE_POSITION);
        }

    }

   void set_servo_down ()
    {
        if (v_servo_puller != null)
        {
            v_servo_puller.setPosition(H_ARM_MAX_RANGE);
        }
        if (v_servo_back != null)
        {
            v_servo_back.setPosition(H_BACK_MIN_RANGE);
        }

    }

    void set_servo_up ()
    {
        if (v_servo_puller != null)
        {
            v_servo_puller.setPosition(H_ARM_MIN_RANGE);
        }
        if (v_servo_back != null)
        {
            v_servo_back.setPosition(H_BACK_MAX_RANGE);
        }

    }

    void hang_forward()
    {
        set_hang_forward();
    }

    void hang_backward()
    {
        set_hang_backward();
    }

    void go_forward()
    {
        set_direction_forward(true);
    }

    void go_backward()
    {
        set_direction_forward(false);
    }

    void set_direction_forward(boolean forward) {
        if (forward) {
            if (v_motor_f_left_drive != null)
                v_motor_f_left_drive.setDirection(DcMotor.Direction.FORWARD);
            if (v_motor_r_left_drive != null)
                v_motor_r_left_drive.setDirection(DcMotor.Direction.FORWARD);
            if (v_motor_f_right_drive != null)
                v_motor_f_right_drive.setDirection (DcMotor.Direction.REVERSE);
            if (v_motor_r_right_drive != null)
                v_motor_r_right_drive.setDirection (DcMotor.Direction.REVERSE);
        } else {
            if (v_motor_f_left_drive != null)
                v_motor_f_left_drive.setDirection(DcMotor.Direction.REVERSE);
            if (v_motor_r_left_drive != null)
                v_motor_r_left_drive.setDirection(DcMotor.Direction.REVERSE);
            if (v_motor_f_right_drive != null)
                v_motor_f_right_drive.setDirection (DcMotor.Direction.FORWARD);
            if (v_motor_r_right_drive != null)
                v_motor_r_right_drive.setDirection (DcMotor.Direction.FORWARD);
        }
    }

    void set_hang_forward() {
        if (v_motor_left_hang != null)
            v_motor_left_hang.setDirection(DcMotor.Direction.REVERSE);
        if (v_motor_right_hang != null)
            v_motor_right_hang.setDirection (DcMotor.Direction.FORWARD);
     }

    void set_hang_backward() {
        if (v_motor_left_hang != null)
            v_motor_left_hang.setDirection(DcMotor.Direction.FORWARD);
        if (v_motor_right_hang != null)
            v_motor_right_hang.setDirection (DcMotor.Direction.REVERSE);
    }

    void turn(boolean left) {
        if (left) {
            if (v_motor_f_left_drive != null)
                v_motor_f_left_drive.setDirection(DcMotor.Direction.REVERSE);
            if (v_motor_r_left_drive != null)
                v_motor_r_left_drive.setDirection(DcMotor.Direction.REVERSE);
            if (v_motor_f_right_drive != null)
                v_motor_f_right_drive.setDirection (DcMotor.Direction.REVERSE);
            if (v_motor_r_right_drive != null)
                v_motor_r_right_drive.setDirection (DcMotor.Direction.REVERSE);
        } else {
            if (v_motor_f_left_drive != null)
                v_motor_f_left_drive.setDirection(DcMotor.Direction.FORWARD);
            if (v_motor_r_left_drive != null)
                v_motor_r_left_drive.setDirection(DcMotor.Direction.FORWARD);
            if (v_motor_f_right_drive != null)
                v_motor_f_right_drive.setDirection (DcMotor.Direction.FORWARD);
            if (v_motor_r_right_drive != null)
                v_motor_r_right_drive.setDirection (DcMotor.Direction.FORWARD);
        }
    }

    void turn_left ()
    {
        turn(true);
    }

    void turn_right ()
    {
        turn(false);
    }

    //--------------------------------------------------------------------------
    //
    // a_warning_generated
    //
    /**
     * Access whether a warning has been generated.
     */
    boolean a_warning_generated ()

    {
        return v_warning_generated;

    } // a_warning_generated

    //--------------------------------------------------------------------------
    //
    // a_warning_message
    //
    /**
     * Access the warning message.
     */
    String a_warning_message ()

    {
        return v_warning_message;

    } // a_warning_message

    //--------------------------------------------------------------------------
    //
    // m_warning_message
    //
    /**
     * Mutate the warning message by ADDING the specified message to the current
     * message; set the warning indicator to true.
     *
     * A comma will be added before the specified message if the message isn't
     * empty.
     */
    void m_warning_message (String p_exception_message)

    {
        if (v_warning_generated)
        {
            v_warning_message += ", ";
        }
        v_warning_generated = true;
        v_warning_message += p_exception_message;

    } // m_warning_message

    //--------------------------------------------------------------------------
    //
    // start
    //
    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     *
     * The system calls this member once when the OpMode is enabled.
     */
    @Override public void start ()

    {
        //
        // Only actions that are common to all Op-Modes (i.e. both automatic and
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.
        //

    } // start

    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Perform any actions that are necessary while the OpMode is running.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override public void loop ()

    {
        //
        // Only actions that are common to all OpModes (i.e. both auto and\
        // manual) should be implemented here.
        //
        // This method is designed to be overridden.
        //

    } // loop

    //--------------------------------------------------------------------------
    //
    // stop
    //
    /**
     * Perform any actions that are necessary when the OpMode is disabled.
     *
     * The system calls this member once when the OpMode is disabled.
     */
    @Override public void stop ()
    {
        //
        // Nothing needs to be done for this method.
        //

    } // stop

    //--------------------------------------------------------------------------
    //
    // scale_motor_power
    //
    /**
     * Scale the joystick input using a nonlinear algorithm.
     */
    float scale_motor_power (float p_power)
    {
        //
        // Assume no scaling.
        //
        float l_scale = 0.0f;

        //
        // Ensure the values are legal.
        //
        float l_power = Range.clip (p_power, -1, 1);

        float[] l_array =
                { 0.00f, 0.05f, 0.09f, 0.10f, 0.12f
                        , 0.15f, 0.18f, 0.24f, 0.30f, 0.36f
                        , 0.43f, 0.50f, 0.60f, 0.72f, 0.85f
                        , 1.00f, 1.00f
                };

        //
        // Get the corresponding index for the specified argument/parameter.
        //
        int l_index = (int)(l_power * 16.0);
        if (l_index < 0)
        {
            l_index = -l_index;
        }
        else if (l_index > 16)
        {
            l_index = 16;
        }

        if (l_power < 0)
        {
            l_scale = -l_array[l_index];
        }
        else
        {
            l_scale = l_array[l_index];
        }

        return l_scale;

    } // scale_motor_power

    //--------------------------------------------------------------------------
    //
    // a_left_drive_power
    //
    /**
     * Access the left drive motor's power level.
     */
    double a_f_left_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_f_left_drive != null)
        {
            l_return = v_motor_f_left_drive.getPower ();
        }

        return l_return;

    } // a_left_drive_power
    double a_r_left_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_r_left_drive != null)
        {
            l_return = v_motor_r_left_drive.getPower ();
        }

        return l_return;

    } // a_left_drive_power

    //--------------------------------------------------------------------------
    //
    // a_right_drive_power
    //
    /**
     * Access the right drive motor's power level.
     */
    double a_f_right_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_f_right_drive != null)
        {
            l_return = v_motor_f_right_drive.getPower ();
        }

        return l_return;

    } // a_right_drive_power

    double a_r_right_drive_power ()
    {
        double l_return = 0.0;

        if (v_motor_r_right_drive != null)
        {
            l_return = v_motor_r_right_drive.getPower ();
        }

        return l_return;

    } // a_right_drive_power

    //--------------------------------------------------------------------------
    //
    // set_drive_power
    //
    /**
     * Scale the joystick input using a nonlinear algorithm.
     */
    void set_drive_power (double p_left_power, double p_right_power)

    {

        if (v_motor_f_left_drive != null)
        {
            v_motor_f_left_drive.setPower (p_left_power);
        }

        if (v_motor_f_right_drive != null)
        {
            v_motor_f_right_drive.setPower (p_right_power);
        }
        if (v_motor_r_left_drive != null)
        {
            v_motor_r_left_drive.setPower (p_left_power);
        }
        if (v_motor_r_right_drive != null)
        {
            v_motor_r_right_drive.setPower (p_right_power);
        }

    } // set_drive_power

    //--------------------------------------------------------------------------
    //
    // run_using_left_drive_encoder
    //
    /**
     * Set the left drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_using_left_drive_encoder ()

    {
        if (v_motor_r_left_drive != null)
        {
            telemetry.addData("10", "run_using_left_drive_encoder");

            v_motor_r_left_drive.setMode
                    (DcMotorController.RunMode.RUN_USING_ENCODERS
                    );
        }

    } // run_using_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_using_right_drive_encoder
    //
    /**
     * Set the right drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_using_right_drive_encoder ()

    {
        if (v_motor_r_right_drive != null)
        {
            telemetry.addData("11", "run_using_right_drive_encoder");
            v_motor_r_right_drive.setMode
                    (DcMotorController.RunMode.RUN_USING_ENCODERS
                    );
        }

    } // run_using_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_using_encoders
    //
    /**
     * Set both drive wheel encoders to run, if the mode is appropriate.
     */
    public void run_using_encoders ()

    {
        //
        // Call other members to perform the action on both motors.
        //
        run_using_left_drive_encoder ();
        run_using_right_drive_encoder ();

    } // run_using_encoders

    public void run_to_target (int ticks)

    {
        reset_drive_encoders();

        int rpos  = v_motor_r_right_drive.getCurrentPosition();
        int lpos  = v_motor_r_left_drive.getCurrentPosition();

        v_motor_r_right_drive.setTargetPosition(ticks);
        v_motor_r_left_drive.setTargetPosition(ticks);



        v_motor_r_right_drive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        v_motor_r_left_drive.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        set_drive_power(0.2, 0.2);

        while(rpos < ticks && lpos < ticks){
            rpos  = v_motor_r_right_drive.getCurrentPosition();
            lpos  = v_motor_r_left_drive.getCurrentPosition();
        }

        telemetry.addData("2 ", "motorLeft:  " + String.format("%d", v_motor_r_left_drive.getTargetPosition()));
        telemetry.addData("3 ", "motorRight:  " + String.format("%d", v_motor_r_right_drive.getTargetPosition()));

        set_drive_power(0, 0);
        reset_drive_encoders();

    } // run_using_encoders

    //--------------------------------------------------------------------------
    //
    // run_without_left_drive_encoder
    //
    /**
     * Set the left drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_without_left_drive_encoder ()

    {
        if (v_motor_r_left_drive != null)
        {
            if (v_motor_r_left_drive.getMode () ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                v_motor_r_left_drive.setMode
                        ( DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }

    } // run_without_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_without_right_drive_encoder
    //
    /**
     * Set the right drive wheel encoder to run, if the mode is appropriate.
     */
    public void run_without_right_drive_encoder ()

    {
        if (v_motor_r_right_drive != null)
        {
            if (v_motor_r_right_drive.getMode () ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                v_motor_r_right_drive.setMode
                        ( DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }

    } // run_without_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // run_without_drive_encoders
    //
    /**
     * Set both drive wheel encoders to run, if the mode is appropriate.
     */
    public void run_without_drive_encoders ()

    {
        //
        // Call other members to perform the action on both motors.
        //
        run_without_left_drive_encoder ();
        run_without_right_drive_encoder ();

    } // run_without_drive_encoders

    //--------------------------------------------------------------------------
    //
    // reset_left_drive_encoder
    //
    /**
     * Reset the left drive wheel encoder.
     */
    public void reset_left_drive_encoder ()

    {
        if (v_motor_r_left_drive != null)
        {
            v_motor_r_left_drive.setMode
                    ( DcMotorController.RunMode.RESET_ENCODERS
                    );
        }

    } // reset_left_drive_encoder

    //--------------------------------------------------------------------------
    //
    // reset_right_drive_encoder
    //
    /**
     * Reset the right drive wheel encoder.
     */
    public void reset_right_drive_encoder ()

    {
        if (v_motor_r_right_drive != null)
        {
            v_motor_r_right_drive.setMode
                    ( DcMotorController.RunMode.RESET_ENCODERS
                    );
        }


    } // reset_right_drive_encoder

    //--------------------------------------------------------------------------
    //
    // reset_drive_encoders
    //
    /**
     * Reset both drive wheel encoders.
     */
    public void reset_drive_encoders ()

    {
        //
        // Reset the motor encoders on the drive wheels.
        //
        reset_left_drive_encoder ();
        reset_right_drive_encoder ();

    } // reset_drive_encoders


    //--------------------------------------------------------------------------
    //
    // a_left_encoder_count
    //
    /**
     * Access the left encoder's count.
     */
    int m_f_left_encoder_count=0;
    int a_f_left_encoder_count ()
    {
        int l_return = 0;

        if (v_motor_f_left_drive != null)
        {
            l_return = v_motor_f_left_drive.getCurrentPosition ();
        }
        m_f_left_encoder_count = l_return;
        return l_return;

    } // a_left_encoder_count

    int m_r_left_encoder_count = 0;
    int a_r_left_encoder_count ()
    {
        int l_return = 0;

        if (v_motor_r_left_drive != null)
        {
            l_return = v_motor_r_left_drive.getCurrentPosition ();
        }
        m_r_left_encoder_count = l_return;
        return l_return;

    } // a_left_encoder_count

    //--------------------------------------------------------------------------
    //
    // a_right_encoder_count
    //
    /**
     * Access the right encoder's count.
     */
    int m_f_right_encoder_count = 0;
    int a_f_right_encoder_count ()

    {
        int l_return = 0;
        if (v_motor_f_right_drive != null)
        {
            l_return = v_motor_f_right_drive.getCurrentPosition ();
        }
        m_f_right_encoder_count = l_return;
        return l_return;

    } // a_right_encoder_count

    int m_r_right_encoder_count = 0;
    int a_r_right_encoder_count ()
    {
        int l_return = 0;

        if (v_motor_r_right_drive != null)
        {
            l_return = v_motor_r_right_drive.getCurrentPosition ();
        }
        m_r_right_encoder_count = l_return;
        return l_return;

    } // a_right_encoder_count

    //--------------------------------------------------------------------------
    //
    // has_left_drive_encoder_reached
    //
    /**
     * Indicate whether the left drive motor's encoder has reached a value.
     */
    boolean has_left_drive_encoder_reached (double p_count)

    {
        //
        // Assume failure.
        //
        boolean l_return = false;
        double cur_val = v_motor_r_left_drive.getCurrentPosition ();
        telemetry.addData("02", "l-encoder" + cur_val);

        if (v_motor_r_left_drive != null)
        {
            //
            // Has the encoder reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (Math.abs (cur_val) > p_count)
            {
                //
                // Set the status to a positive indication.
                //
                l_return = true;
            }
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_left_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // has_right_drive_encoder_reached
    //
    /**
     * Indicate whether the right drive motor's encoder has reached a value.
     */
    boolean has_right_drive_encoder_reached (double p_count)

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        if (v_motor_r_right_drive != null)
        {
            double cur_val = v_motor_r_right_drive.getCurrentPosition ();
            telemetry.addData("01", "r-encoder" + cur_val);
            //
            // Have the encoders reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (Math.abs (cur_val) > p_count)
            {
                //
                // Set the status to a positive indication.
                //
                l_return = true;
            }
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_right_drive_encoder_reached

    void reset_encoder(DcMotor motor) {
        if (motor != null) {
            motor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        }
    }

    void set_encoder(DcMotor motor) {
        if (motor != null) {
            motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        }
    }

    boolean has_encoder_reached_count (DcMotor motor, double p_count)
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        if (motor != null)
        {
            double cur_val = motor.getCurrentPosition ();
            telemetry.addData("01", "encoder" + cur_val);
            //
            // Have the encoders reached the specified values?
            //
            // TODO Implement stall code using these variables.
            //
            if (Math.abs (cur_val) > p_count)
            {
                l_return = true;
            }
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_right_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reached
    //
    /**
     * Indicate whether the drive motors' encoders have reached a value.
     */
    boolean have_drive_encoders_reached
    ( double p_left_count
            , double p_right_count
    )

    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Have the encoders reached the specified values?
        //
        if (has_left_drive_encoder_reached (p_left_count) &&
            has_right_drive_encoder_reached (p_right_count))
        {
             l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // have_encoders_reached

    //========================================================================
    //
    // Methods for the hang motors
    //
    /**
     * Indicate whether the drive motors' encoders have reached a value.
     */
    void hang_forward_no_encoder(
            double power)
    {
        if (v_motor_left_hang != null)
        {
            if (v_motor_left_hang.getMode () ==
                    DcMotorController.RunMode.RESET_ENCODERS)
            {
                v_motor_left_hang.setMode
                        ( DcMotorController.RunMode.RUN_WITHOUT_ENCODERS
                        );
            }
        }
        set_hang_power(power);
    }

    public void set_hang_encoders ()
    {
        set_encoder(v_motor_left_hang);
        set_encoder(v_motor_right_hang);
    }

    public void reset_hang_encoders ()
    {
        reset_encoder(v_motor_left_hang);
        reset_encoder(v_motor_right_hang);
    } // reset_hang_encoders

    void set_hang_power(double power){
        if (v_motor_left_hang != null)
            v_motor_left_hang.setPower (power);

        if (v_motor_right_hang != null)
            v_motor_right_hang.setPower (power);
    }

    protected boolean has_hang_moved(double degrees, double power) {
        set_hang_power(power);
        if (have_hang_encoders_reached(convertHangerDegreesToTicks(degrees))) {
            reset_hang_encoders();
            set_hang_power(0.0f);
            return true;
        }
        return false;
    }

    boolean have_hang_encoders_reached ( double p_count  )
    {
        //
        // Assume failure.
        //
        boolean l_return = false;
        if (has_encoder_reached_count(v_motor_left_hang,  p_count) &&
            has_encoder_reached_count(v_motor_right_hang, p_count))
        {
            l_return = true;
        }
        return l_return;
    } // have_encoders_reached
    //========================================================================


    void drive_forward_no_encoder(
            double p_left_power, double p_right_power)
    {
        go_forward();
        set_drive_power(p_left_power,p_right_power);
    }
    //--------------------------------------------------------------------------
    //
    // drive_using_encoders
    //
    /**
     * Indicate whether the drive motors' encoders have reached a value.
     */
    boolean drive_using_encoders
    ( double p_left_power
            , double p_right_power
            , double p_left_count
            , double p_right_count
    )

    {
        //
        // Assume the encoders have not reached the limit.
        //
        boolean l_return = false;

        //
        // Tell the system that motor encoders will be used.
        //
        run_using_encoders ();

        //
        // Start the drive wheel motors at full power.
        //
        set_drive_power (p_left_power, p_right_power);

        //
        // Have the motor shafts turned the required amount?
        //
        // If they haven't, then the op-mode remains in this state (i.e this
        // block will be executed the next time this method is called).
        //
        if (have_drive_encoders_reached (p_left_count, p_right_count))
        {
            //
            // Reset the encoders to ensure they are at a known good value.
            //
            reset_drive_encoders ();

            //
            // Stop the motors.
            //
            set_drive_power (0.0f, 0.0f);

            //
            // Transition to the next state when this method is called
            // again.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // drive_using_encoders

    //--------------------------------------------------------------------------
    //
    // has_left_drive_encoder_reset
    //
    /**
     * Indicate whether the left drive encoder has been completely reset.
     */
    boolean has_left_drive_encoder_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the left encoder reached zero?
        //
        if (a_r_left_encoder_count() == 0)
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_left_drive_encoder_reset

    //--------------------------------------------------------------------------
    //
    // has_right_drive_encoder_reset
    //
    /**
     * Indicate whether the left drive encoder has been completely reset.
     */
    boolean has_right_drive_encoder_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Has the right encoder reached zero?
        //
        if (a_r_right_encoder_count() == 0)
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // has_right_drive_encoder_reset

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reset
    //
    /**
     * Indicate whether the encoders have been completely reset.
     */
    boolean have_drive_encoders_reset ()
    {
        //
        // Assume failure.
        //
        boolean l_return = false;

        //
        // Have the encoders reached zero?
        //
        if (has_left_drive_encoder_reset() && has_right_drive_encoder_reset ())
        {
            //
            // Set the status to a positive indication.
            //
            l_return = true;
        }

        //
        // Return the status.
        //
        return l_return;

    } // have_drive_encoders_reset

    boolean have_hang_encoders_reset ()
    {
        int lcnt = v_motor_left_hang.getCurrentPosition ();
        int rcnt = v_motor_right_hang.getCurrentPosition ();
        telemetry.addData("Hang position: ", lcnt + ", " + rcnt);
        return lcnt == 0 && rcnt == 0;
    }
    //--------------------------------------------------------------------------
    //
    // a_puller_power
    //
    /**
     * Access the left arm motor's power level.
     */
    double a_puller_power ()
    {
        double l_return = 0.0;

        if (v_motor_puller != null)
        {
            l_return = v_motor_puller.getPower ();
        }

        return l_return;

    } // a_puller_power

    //--------------------------------------------------------------------------
    //
    // m_puller_power
    //
    /**
     * Access the left arm motor's power level.
     */
    void m_puller_power (double p_level)
    {
        if (v_motor_puller != null)
        {
            v_motor_puller.setPower (p_level);
        }

    } // m_puller_power

    //--------------------------------------------------------------------------
    //
    // a_puller_power
    //
    /**
     * Access the left arm motor's power level.
     */
    double a_slider_power ()
    {
        double l_return = 0.0;

        if (v_motor_slider != null)
        {
            l_return = v_motor_slider.getPower ();
        }

        return l_return;

    } // a_puller_power

    //--------------------------------------------------------------------------
    //
    // m_slider_power
    //
    /**
     * Access the left arm motor's power level.
     */
    void m_slider_power (double p_level)
    {
        if (v_motor_slider != null)
        {
            v_motor_slider.setPower (p_level);
        }

    } // m_slider_power

    //--------------------------------------------------------------------------
    //
    // a_hand_position
    //
    /**
     * Access the hand position.
     */
    double a_puller_position ()
    {
        double l_return = 0.0;

        if (v_servo_puller != null)
            l_return = v_servo_puller.getPosition ();
        return l_return;
    } // a_hand_position


    protected boolean has_driver_forward(double count, double power) {
        set_drive_power(power, power);

        //
        // Have the motor shafts turned the required amount?
        //
        // If they haven't, then the op-mode remains in this state (i.e this
        // block will be executed the next time this method is called).
        //
        if (have_drive_encoders_reached(count, count)) {
            //
            // Reset the encoders to ensure they are at a known good value.
            //
            reset_drive_encoders();
            set_drive_power(0.0f, 0.0f);

            return true;
        }
        return false;
    }

    protected boolean has_driver_backward(double count, double power) {
        set_drive_power(power, power);

        //
        // Have the motor shafts turned the required amount?
        //
        // If they haven't, then the op-mode remains in this state (i.e this
        // block will be executed the next time this method is called).
        //
        if (have_drive_encoders_reached(count, count)) {
            //
            // Reset the encoders to ensure they are at a known good value.
            //
            reset_drive_encoders();
            set_drive_power(0.0f, 0.0f);
            return true;
        }
        return false;
    }

    protected boolean has_driver_forward_cm(double cm, double power) {
        return has_driver_forward(convertCMToTicks(cm), power);
    }

    protected boolean has_driver_turned_degrees(double degrees, double power){
        return has_driver_turned(convertDegreesToTicks(degrees), power);
    }

    protected boolean has_driver_turned(double count, double power) {

        set_drive_power(power, power);
        telemetry.addData("6 - ", "turn ticks=" + count + "p=" + power);

        //
        // Have the motor shafts turned the required amount?
        //
        // If they haven't, then the op-mode remains in this state (i.e this
        // block will be executed the next time this method is called).
        //
        if (has_right_drive_encoder_reached (count) &&
                has_left_drive_encoder_reached (count)) {
            //
            // Reset the encoders to ensure they are at a known good value.
            //
            reset_drive_encoders();
            set_drive_power(0.0f, 0.0f);
            return true;
        }
        return false;
    }

    protected long convertCMToTicks(double cent){
        return Math.round((cent / WHEEL_CIRCUMFERENCE) * TICKS_PER_REV);
    }

    protected double convertDegreesToTicks(double degrees){
        return Math.round((degrees * TURN_45_DEGREES / 45));
    }


    protected double convertHangerDegreesToTicks(double degrees){
        return Math.round((degrees * HANGER_45_DEGREES / 45));
    }
    //--------------------------------------------------------------------------
    //
    // v_warning_generated
    //
    /**
     * Indicate whether a message is a available to the class user.
     */
    private boolean v_warning_generated = false;

    //--------------------------------------------------------------------------
    //
    // v_warning_message
    //
    /**
     * Store a message to the user if one has been generated.
     */
    private String v_warning_message;



    //--------------------------------------------------------------------------
    //
    // v_motor_f_left_drive
    //
    /**
     * Manage the aspects of the front left drive motor.
     */
    private DcMotor v_motor_f_left_drive;

    //--------------------------------------------------------------------------
    //
    // v_motor_f_right_drive
    //
    /**
     * Manage the aspects of the front right drive motor.
     */
    private DcMotor v_motor_f_right_drive;


    //--------------------------------------------------------------------------
    //
    // v_motor_r_left_drive
    //
    /**
     * Manage the aspects of the rearleft drive motor.
     */
    private DcMotor v_motor_r_left_drive;

    //--------------------------------------------------------------------------
    //
    // v_motor_r_right_drive
    //
    /**
     * Manage the aspects of the rear right drive motor.
     */
    private DcMotor v_motor_r_right_drive;

    //--------------------------------------------------------------------------
    //
    // v_motor_r_left_drive
    //
    /**
     * Manage the aspects of the rearleft drive motor.
     */
    private DcMotor v_motor_left_hang;

    //--------------------------------------------------------------------------
    //
    // v_motor_r_right_drive
    //
    /**
     * Manage the aspects of the rear right drive motor.
     */
    private DcMotor v_motor_right_hang;

    //--------------------------------------------------------------------------
    //
    // v_motor_puller
    //
    /**
     * Motor to pull the tape.
     */
    private DcMotor v_motor_puller;

    //--------------------------------------------------------------------------
    //
    // v_motor_slider
    //
    /**
     * Motor to control the slider to get the zip line.
     */
    private DcMotor v_motor_slider;


     //--------------------------------------------------------------------------
    //
    // v_servo_puller
    //
    /**
     * Use to guide the puller angle.
     */
    private Servo v_servo_puller;

    //--------------------------------------------------------------------------
    //
    // v_servo_back
    //
    /**
     * Use to control the back plate
     */
    private Servo v_servo_back;

    //--------------------------------------------------------------------------
    //
    // v_servo_climber
    //
    /**
     * Use to drop the climber
     */
    private Servo v_servo_climber;

    //--------------------------------------------------------------------------
    //
    // v_servo_climber
    //
    /**
     * Use to position the pushing pad
     */
    private Servo v_servo_beacon;

    // ODS sensor to scan tape
    private OpticalDistanceSensor v_ods;

    private UltrasonicSensor v_ultra;

    private TouchSensor v_touch;

    private ColorSensor v_color;


} // PushBotHardware
