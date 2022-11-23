package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * This file
 */

@TeleOp(name="Colour Sensor", group="Linear Opmode")
//@Disabled
public class coloursensor extends LinearOpMode {

  // Declare OpMode members.
  ColorSensor sensorColor;

  float hsvValues[] = {0F, 0F, 0F};
  final float values[] = hsvValues;
  final double SCALE_FACTOR = 255;

  @Override
  public void runOpMode() {

    sensorColor = hardwareMap.get(ColorSensor.class, "colourSensor");

    waitForStart();

    while (opModeIsActive()) {

      Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
              (int) (sensorColor.green() * SCALE_FACTOR),
              (int) (sensorColor.blue() * SCALE_FACTOR),
              hsvValues);

      telemetry.addData("hue", hsvValues[0]);
      telemetry.addData("Sat", hsvValues[1]);
      telemetry.addData("val", hsvValues[2]);
      telemetry.update();


      if (hsvValues[0] < 150 && hsvValues[0] > 140) {
        telemetry.addLine("This is green");
      } else {
        telemetry.addLine("This is not green: Wrong Hue");
        telemetry.update();
      }

      if (hsvValues[0] < 30 && hsvValues[0] > 10) {
        telemetry.addLine("This is red");
      } else {
        telemetry.addLine("This is not red: Wrong Hue");
        telemetry.update();
      }

      if (hsvValues[0] < 220 && hsvValues[0] > 190) {
        if (hsvValues[1] < 0.6 && hsvValues[1] > 0.4) {
          telemetry.addLine("This is purple");
        } else {
          telemetry.addLine("This is not purple: Wrong Hue");
          telemetry.update();
        }
      }
    }
  }
}
