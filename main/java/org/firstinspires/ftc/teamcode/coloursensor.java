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
@Disabled
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

      /*
            if (hsvValues[0] < 50 && hsvValues[0] > 30){
                if (hsvValues[1] > 0.6){
                    if (hsvValues[2] > 200){
                        telemetry.addLine("This could be red");
                        telemetry.update();
                    } else{
                        telemetry.addLine("This is not red: Wrong Value");
                        telemetry.update();
                    }
                } else{
                    telemetry.addLine("This is not red: Wrong Saturation");
                    telemetry.update();
                }
            } else{
                telemetry.addLine("This is not red: Wrong Hue");
                telemetry.update();
                }
                */

    }
  }
}