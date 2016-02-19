/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.Ftc9015;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

public class T9015ColorSensorDriver extends OpMode
{
  public enum ColorSensorDevice {ADAFRUIT, HITECHNIC_NXT, MODERN_ROBOTICS_I2C};

  //public ColorSensorDevice device = ColorSensorDevice.MODERN_ROBOTICS_I2C;
  public ColorSensorDevice device = ColorSensorDevice.MODERN_ROBOTICS_I2C;

  FtcConfig ftcConfig=new FtcConfig();
  ColorSensor colorSensor;
  DeviceInterfaceModule cdim;
  LED led;
  TouchSensor t;
  OpticalDistanceSensor ods;
  double  ods_val;
  boolean led_on = false;

  UltrasonicSensor ultra;
  double us_value;
  int n = 0;


  private Servo s_beacon, s_pull;

  int ods_int;
  @Override public void init ()

  {

/*
    //hardwareMap.logDevices();
    telemetry.addData("00", "init");
    s_beacon = hardwareMap.servo.get("sbeacon");
    s_pull = hardwareMap.servo.get("spull");
    telemetry.addData("00a", "init sbeacon");
    s_pull.setPosition(0.95);
      s_beacon.setPosition(RobotInfo.BEACON_LEFT);

    //cdim = hardwareMap.deviceInterfaceModule.get("dim");

    telemetry.addData("01", "init" + "device " + device);

    switch (device) {
      case HITECHNIC_NXT:
        colorSensor = hardwareMap.colorSensor.get("nxt");
        telemetry.addData("02", "HITECHNIC_NXT " + colorSensor);
        break;
      case ADAFRUIT:
        colorSensor = hardwareMap.colorSensor.get("ada");
        telemetry.addData("02", "ADAFRUIT " + colorSensor);
        break;
      case MODERN_ROBOTICS_I2C:
        colorSensor = hardwareMap.colorSensor.get("mr_color");
        enableLed(false);
        telemetry.addData("02", "MODERN_ROBOTICS_I2C " + colorSensor);
        break;
    }

   // mrc = new T9015ColorSensor();
   // mrc.init(false);


    t = hardwareMap.touchSensor.get("touch");
    if (t!=null) {
      led_on = t.isPressed();
      telemetry.addData("03", "touch=" + led_on);
      enableLed(led_on);
    }
*/

    //led = hardwareMap.led.get("led");
    //if (led != null) {
    //  led.enable(led_on);
    //}

    ods = hardwareMap.opticalDistanceSensor.get("ods");
    if (ods != null) {
      ods.enableLed(false);
      ods_val = ods.getLightDetected();
      ods_int = ods.getLightDetectedRaw();
      telemetry.addData("03a", "ods=" + ods_int + " " + ods_val);
    }

    ultra = hardwareMap.ultrasonicSensor.get("ultra");
    if (ultra !=null) {
      us_value = ultra.getUltrasonicLevel();
      telemetry.addData("04", "ultra=" + us_value);
    }

      ftcConfig.init(hardwareMap.appContext, this);
      telemetry.addData("ColorIsRed", Boolean.toString(ftcConfig.param.colorIsRed));
      telemetry.addData("DelayInSec", Integer.toString(ftcConfig.param.delayInSec));
      telemetry.addData("AutonType", ftcConfig.param.autonType);

  }

  @Override public void start ()

  {
    //
    // Only actions that are common to all Op-Modes (i.e. both automatic and
    // manual) should be implemented here.
    //
    // This method is designed to be overridden.
    //

  } // start

  float hsvValues[] = {0,0,0};
  final float values[] = hsvValues;
//  final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(R.id.RelativeLayout);


  @Override public void loop ()
  {

    telemetry.addData("04", "loop" + n);
    n=n+1;
    /*
    if (t!=null) {
      led_on = t.isPressed();
      telemetry.addData("03", "touch=" + led_on);
      enableLed(led_on);
    }
*/
/*
    enableLed(false);

    switch (device) {
        case HITECHNIC_NXT:
          Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
          break;
        case ADAFRUIT:
          Color.RGBToHSV((colorSensor.red() * 255) / 800, (colorSensor.green() * 255) / 800, (colorSensor.blue() * 255) / 800, hsvValues);
          break;
        case MODERN_ROBOTICS_I2C:
          Color.RGBToHSV(colorSensor.red()*8, colorSensor.green()*8, colorSensor.blue()*8, hsvValues);

          break;
    }
*/

    if (ods!=null) {
        ods.enableLed(true);
      ods_val = ods.getLightDetected();
      ods_int = ods.getLightDetectedRaw();
      telemetry.addData("03a", "ods=" + ods_int + " " + ods_val);
    }
    if (ultra !=null) {
      us_value = ultra.getUltrasonicLevel();
      telemetry.addData("04", "ultra=" + us_value);
    }
      //ftcConfig.init(hardwareMap.appContext, this);
      telemetry.addData("ColorIsRed", Boolean.toString(ftcConfig.param.colorIsRed));
      telemetry.addData("DelayInSec", Integer.toString(ftcConfig.param.delayInSec));
      telemetry.addData("AutonType", ftcConfig.param.autonType);
/*
      telemetry.addData("Clear", colorSensor.alpha());
      telemetry.addData("Red  ", colorSensor.red());
      telemetry.addData("Green", colorSensor.green());
      telemetry.addData("Blue ", colorSensor.blue());
      telemetry.addData("Hue", hsvValues[0]);
*/
/*
    mrc.get_color();
    telemetry.addData("mClear", mrc.alpha);
    telemetry.addData("mRed  ", mrc.red);
    telemetry.addData("mGreen", mrc.green);
    telemetry.addData("mBlue ", mrc.blue);
    telemetry.addData("mHue", mrc.hsvValues[0]);

*/
  /*  //<editor-fold desc="Description">
    if ( (colorSensor.blue()>=1))
    {
      s_beacon.setPosition(RobotInfo.BEACON_RIGHT);
    }
    else
    {
      s_beacon.setPosition(RobotInfo.BEACON_LEFT);
    }
    //</editor-fold>
*/


//      relativeLayout.post(new Runnable() {
//        public void run() {
//          relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
//        }
//      });

  }

  @Override public void stop ()
  {
    //
    // Nothing needs to be done for this method.
    //

  } // stop

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