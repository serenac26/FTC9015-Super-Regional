package com.qualcomm.ftcrobotcontroller.Ftc9015;

import android.graphics.Color;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by ttn on 02/11/2016.
 */
public class T9015ColorSensor {

    public enum ColorSensorDevice {ADAFRUIT, HITECHNIC_NXT, MODERN_ROBOTICS_I2C};

    public ColorSensorDevice device = ColorSensorDevice.MODERN_ROBOTICS_I2C;
    //public ColorSensorDevice device;

    HardwareMap hardwareMap;
    ColorSensor colorSensor;
    DeviceInterfaceModule cdim;
    LED led;

    int n = 0;
    public T9015ColorSensor()
    {

    }

    public void init (boolean active)
    {
        //device = color_dev;
        //hardwareMap.logDevices();
        //telemetry.addData("00", "init");
        //telemetry.addData("01", "init" + "device " + device);

        switch (device) {
            case HITECHNIC_NXT:
                colorSensor = hardwareMap.colorSensor.get("nxt");
                //telemetry.addData("02", "HITECHNIC_NXT " + colorSensor);
                break;
            case ADAFRUIT:
                colorSensor = hardwareMap.colorSensor.get("ada");
                //telemetry.addData("02", "ADAFRUIT " + colorSensor);
                break;
            case MODERN_ROBOTICS_I2C:
                colorSensor = hardwareMap.colorSensor.get("mr_color");
                colorSensor.enableLed(active);
                //telemetry.addData("02", "MODERN_ROBOTICS_I2C " + colorSensor);
                break;
        }

        enableLed(active);
    } //

    float hsvValues[] = {0,0,0};
    final float values[] = hsvValues;
    public int red;
    public int blue;
    public int green;
    public int alpha;

    public void get_color() {

        switch (device) {
            case HITECHNIC_NXT:
                Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
                break;
            case ADAFRUIT:
                Color.RGBToHSV((colorSensor.red() * 255) / 800, (colorSensor.green() * 255) / 800, (colorSensor.blue() * 255) / 800, hsvValues);
                break;
            case MODERN_ROBOTICS_I2C:
                Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
                break;
        }
        red   = colorSensor.red();
        blue  = colorSensor.blue();
        green = colorSensor.green();
        alpha = colorSensor.alpha();

    }

    boolean is_blue()
    {
        return ((alpha==1) && (blue == 1));
    }

    boolean is_red()
    {
        return ((alpha==1) && (red == 1));
    }


    private void enableLed(boolean value) {
        switch (device) {
            case HITECHNIC_NXT:
                colorSensor.enableLed(value);
                break;
            case ADAFRUIT:
                if (led != null) {
                    led.enable(value);
                }
                break;
            case MODERN_ROBOTICS_I2C:
                colorSensor.enableLed(value);
                break;
        }
    }
}