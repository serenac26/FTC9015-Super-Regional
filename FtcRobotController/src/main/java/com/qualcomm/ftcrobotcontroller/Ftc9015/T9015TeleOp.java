package com.qualcomm.ftcrobotcontroller.Ftc9015;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Minh-Phong on 12/05/2015.
 */

/**
 * Team 9015 TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class T9015TeleOp extends OpMode { // T9015Hardware { // OpMode {

    /*
     * Note: the configuration of the servos is such that
     * as the arm servo approaches 0, the arm position moves up (away from the floor).
     * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
     */
    // TETRIX VALUES.


    final static double ARM_MIN_RANGE  = 0.57;
    final static double ARM_MAX_RANGE  = 0.95;

    final static double BACK_DOWN  = 0.001;
    final static double BACK_UP    = 0.999;

    final static double CLIMBER_START  = 0.999;
    final static double CLIMBER_END    = 0.001;

    boolean lock_puller = false;

    // position of the arm servo.
    double armPosition = 0.0;
    double climberPosition = 0.0;

    // position of back servo
    double backPosition = 0.0;
    double backDelta = 0.1;

    // amount to change the arm servo position.
    double armDelta = 0.005;

    DcMotor motorRightF;
    DcMotor motorLeftF;
    DcMotor motorRightR;
    DcMotor motorLeftR;
    DcMotor hang1;
    DcMotor hang2;
    Servo   spull;
    Servo   sback;
    Servo   sclimber;

    DcMotor pull;
    DcMotor slide;

    double  ods_val;

    int counter = 0;


    OpticalDistanceSensor ods;


    /**
     * Constructor
     */
    public T9015TeleOp() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */


        //super.init();

        motorRightF = hardwareMap.dcMotor.get("motor2");
        motorLeftF = hardwareMap.dcMotor.get("motor1");
        motorRightF.setDirection(DcMotor.Direction.REVERSE);

        motorRightR = hardwareMap.dcMotor.get("motor3");
        motorLeftR = hardwareMap.dcMotor.get("motor4");
        motorLeftR.setDirection(DcMotor.Direction.REVERSE);

        hang1 = hardwareMap.dcMotor.get("hang1");
        hang2 = hardwareMap.dcMotor.get("hang2");
        hang2.setDirection(DcMotor.Direction.REVERSE);

        // servo to guide the puller tape
        spull = hardwareMap.servo.get("spull");

        // back plate servo
        sback = hardwareMap.servo.get("sback");

        // back plate servo
        sclimber = hardwareMap.servo.get("sclimber");
        sclimber.setPosition(RobotInfo.CLIMBER_LOCK_POSITION);

        // motor for the puller tape
        pull = hardwareMap.dcMotor.get("pull");

        // motor for the slider in the back
        slide = hardwareMap.dcMotor.get("slide");

        //ods = hardwareMap.opticalDistanceSensor.get("ods");
        //ods.enableLed(true);



        // assign the starting position of the puller
        armPosition = spull.getPosition();

        // init position for the back plate servo is down
        backPosition = sback.getPosition();

        telemetry.addData("Text", "*** 9015 Robot Init Data***");
        telemetry.addData("i01", "arm:  " + String.format("%.2f", armPosition));
        telemetry.addData("i02", "climber: " + String.format("%.2f ", climberPosition) );
        telemetry.addData("i03", "back: " + String.format("%.2f", backPosition));
     //   telemetry.addData("i04", "ods: " + String.format("%.2f", ods_val));
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {


		/*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * wrist/claw via the a,b, x, y buttons
		 */

            // power for the hangers motors
            double hpower;

            // note that if y equal -1 then joystick is pushed all of the way forward.
            float left = -gamepad1.left_stick_y;
            float right = -gamepad1.right_stick_y;

            // clip the right/left values so that the values never exceed +/- 1
            right = Range.clip(right, -1, 1);
            left = Range.clip(left, -1, 1);

            // scale the joystick value to make it easier to control
            // the robot more precisely at slower speeds.
            right = (float) scaleInput(right);
            left = (float) scaleInput(left);

            // write the values to the motors
            motorRightF.setPower(right);
            motorLeftF.setPower(left);
            motorRightR.setPower(left);
            motorLeftR.setPower(right);

            // moves slider, x to the left, b to the right
            if (gamepad1.x || gamepad2.x) {
                slide.setPower(-1);
            } else if (gamepad1.b || gamepad2.b) {
                slide.setPower(1);
            } else {
                slide.setPower(0);
            }

            //moves hanging arms
            if (gamepad1.dpad_up) {
                hpower = -0.20;
            } else if (gamepad1.dpad_down) {
                hpower = 0.20;
            } else {
                // power 0
                hpower = 0;
            }
            hang1.setPower(hpower);
            hang2.setPower(hpower);

            //lock winch, if left bumper clicked set motor to 0.25 power constant
            if (gamepad1.left_bumper) {
                if (lock_puller == false) {
                    lock_puller = true;
                } else {
                    lock_puller = false;
                }
            }

            if (lock_puller == true) {
                pull.setPower(0.25);
            } else {
                // moves puller, x is up, y is down
                if (gamepad1.y) {
                    pull.setPower(-0.5);
                } else if (gamepad1.a) {
                    pull.setPower(1);
                } else {
                    pull.setPower(0);
                }
            }

            // update the position of the arm.

            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                // if the bumper is pushed on gamepad1, increment the position of
                // the arm servo.
                armPosition -= armDelta;
            }

            if (gamepad1.right_trigger > 0.25 || gamepad2.right_trigger > 0.25) {
                // if the trigger is pushed on gamepad1, decrease the position of
                // the arm servo.
                armPosition += armDelta;
            }

            if (gamepad1.dpad_right || gamepad2.dpad_down) {
                // if the bumper is pushed on gamepad1, increment the position of
                // the arm servo.
                backPosition = BACK_DOWN;
            }

            if (gamepad1.dpad_left || gamepad2.dpad_up) {
                // if the trigger is pushed on gamepad1, decrease the position of
                // the arm servo.
                backPosition = BACK_UP;
            }
            // write position values to the wrist and claw servo
            sback.setPosition(backPosition);

            // clip the position values so that they never exceed their allowed range.
            armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
            // clawPosition = Range.clip(clawPosition, CLAW_MIN_RANGE, CLAW_MAX_RANGE);

            // write position values to the wrist and claw servo
            spull.setPosition(armPosition);
            //claw.setPosition(clawPosition);

            // clip the position values so that they never exceed their allowed range.
            backPosition = Range.clip(backPosition, BACK_DOWN, BACK_UP);


            climberPosition = sclimber.getPosition();
            if (gamepad2.y) {
                // if the gamepad2.y, drop the climber
                sclimber.setPosition(RobotInfo.CLIMBER_LOCK_POSITION);
            } else if (gamepad2.a) {
                // if the gamepad2.y, drop the climber
                sclimber.setPosition(RobotInfo.CLIMBER_RELEASE_POSITION);
            } else {
                sclimber.setPosition(climberPosition);
            }


		/*
		 * Send telemetry data back to driver station. Note that if we are using
		 * a legacy NXT-compatible motor controller, then the getPower() method
		 * will return a null value. The legacy NXT-compatible motor controllers
		 * are currently write only.
		 */
          //  ods_val = ods.getLightDetectedRaw();



        telemetry.addData("Text", "*** 9015 Robot Data***");
        telemetry.addData("01", "arm:  " + String.format("%.2f", armPosition));
        telemetry.addData("02", "pwr left: " + String.format("%.2f ", left) +
                                "right: " + String.format("%.2f", right));
        telemetry.addData("03", "pwr hang: " + String.format("%.2f", hpower));
//        telemetry.addData("04", "ods: " + String.format("%.2f", ods_val));
        telemetry.addData("i01", "arm:  " + String.format("%.2f", armPosition));
//        telemetry.addData("i02", "climber: " + String.format("%.2f ", climberPosition) );
        telemetry.addData("i03", "back: " + String.format("%.2f", backPosition));

    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

}