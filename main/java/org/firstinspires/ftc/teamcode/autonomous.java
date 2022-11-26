/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Robo Ducks Autonomous", group="Linear Opmode")
public class autonomous extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    public DcMotor  elevator = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    double clawOffset = 0;

    int maximumElevatorHeight = -1111;
    int minimumElevatorHeight = 0;

    int Target = 766; //motor 28 counts and gearbox

    ColorSensor sensorColor;

    int location;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;


    BNO055IMU imu;
    Orientation angles;

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 28 ;
    static final double     DRIVE_GEAR_REDUCTION    = 3.61 * 5.23 ; //3.61 for 4:1
    static final double     WHEEL_DIAMETER_MM   = 96 ;
    static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_MM * Math.PI);




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        elevator = hardwareMap.get(DcMotor.class, "elevator");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("CurrentLeftFrontPos", leftFront.getCurrentPosition() );
        telemetry.addData("CurrentLeftBackPos", leftBack.getCurrentPosition() );
        telemetry.addData("CurrentRightFrontPos", rightFront.getCurrentPosition() );
        telemetry.addData("CurrentRightBackPos", rightBack.getCurrentPosition() );
        telemetry.addData("CurrentElevatorPos", elevator.getCurrentPosition());
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setTargetPosition(0);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        //close claw
        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0.5);
        sleep(1000);

        elevatorUp();
        driveforward(610, 0.2);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        runtime.reset();
        while (runtime.time()< 1){
            getColour();
        }

        goLocation(0.2, location);

        while(1==1 && opModeIsActive()){

            telemetry.addData("hi there", true);
            telemetry.update();

        }
        driveforward(500, 0.2);
        driveforward(-500, 0.2);
        elevatorDown();

       /* driveforward(800, 0.6);
        sleep(1000);
        driveforward(-800, 0.6);
        sleep(1000);
        strafing(800, 0.6, true);
        sleep(1000);
        strafing(-800, 0.6, false);
        sleep(1000);



        //claw closing
        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0.5); */

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);


    }

    void strafing(int Target, double Speed, boolean Direction){

        resetEncoders();

        if (Direction = true) { //right

            int newLeftFrontTarget = leftFront.getCurrentPosition() + Target;
            int newLeftBackTarget = leftBack.getCurrentPosition() - Target;
            int newRightFrontTarget = rightFront.getCurrentPosition() - Target;
            int newRightBackTarget = rightBack.getCurrentPosition() + Target;

            leftFront.setTargetPosition(newLeftFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            rightBack.setTargetPosition(newRightBackTarget);
        }
        else if (Direction = false) { //left
            int newLeftFrontTarget = leftFront.getCurrentPosition() - Target;
            int newLeftBackTarget = leftBack.getCurrentPosition() + Target;
            int newRightFrontTarget = rightFront.getCurrentPosition() + Target;
            int newRightBackTarget = rightBack.getCurrentPosition() - Target;

            leftFront.setTargetPosition(newLeftFrontTarget);
            leftBack.setTargetPosition(newLeftBackTarget);
            rightFront.setTargetPosition(newRightFrontTarget);
            rightBack.setTargetPosition(newRightBackTarget);

        }


        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(Speed);
        leftBack.setPower(Speed);
        rightFront.setPower(Speed);
        rightBack.setPower(Speed);
        int count = 0;

        while (opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy() ) {
            telemetry.addData("Count:", count++);
            telemetry.addData("CurrentLeftFrontPos", leftFront.getCurrentPosition() );
            telemetry.addData("CurrentLeftBackPos", leftBack.getCurrentPosition() );
            telemetry.addData("CurrentRightFrontPos", rightFront.getCurrentPosition() );
            telemetry.addData("CurrentRightBackPos", rightBack.getCurrentPosition() );
            telemetry.update();

        }
        stopmotors();

    }

    void driveforward(int Distance, double Speed){

        resetEncoders();

        int target = (int)(Distance * COUNTS_PER_MM);
        double  steer;

        /*
        int newLeftFrontTarget = leftFront.getCurrentPosition() + Target;
        int newLeftBackTarget = leftBack.getCurrentPosition() + Target;
        int newRightFrontTarget = rightFront.getCurrentPosition() + Target;
        int newRightBackTarget = rightBack.getCurrentPosition() + Target;
         */

        leftFront.setTargetPosition(target);
        leftBack.setTargetPosition(target);
        rightFront.setTargetPosition(target);
        rightBack.setTargetPosition(target);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int count = 0;
        while (opModeIsActive() && leftFront.isBusy() && leftBack.isBusy() && rightFront.isBusy() && rightBack.isBusy() ) {
            /*
            telemetry.addData("Count:", count++);
            telemetry.addData("leftError", newLeftFrontTarget - leftFront.getCurrentPosition());
            telemetry.addData("CurrentLeftFrontPos", leftFront.getCurrentPosition() );
            telemetry.addData("CurrentLeftBackPos", leftBack.getCurrentPosition() );
            telemetry.addData("CurrentRightFrontPos", rightFront.getCurrentPosition() );
            telemetry.addData("CurrentRightBackPos", rightBack.getCurrentPosition() );

             */
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            telemetry.addData("Angle", angles.firstAngle);
            telemetry.update();


            double error = 0 - angles.firstAngle;
            double kp = 0.1;
            double p = error * kp * 0.3;

            if (Distance > 0) {     //forward
                leftFront.setPower(Speed - p);
                leftBack.setPower(Speed - p);
                rightFront.setPower(Speed + p);
                rightBack.setPower(Speed + p);

            }
            else if (Distance < 0){     //backward
                leftFront.setPower(Speed + p);
                leftBack.setPower(Speed + p);
                rightFront.setPower(Speed - p);
                rightBack.setPower(Speed - p);
            }
        }


        stopmotors();

    }

    public void elevatorUp(){

        elevator.setTargetPosition(maximumElevatorHeight);
        elevator.setPower(0.4);

    }

    public void elevatorDown(){

        elevator.setTargetPosition(minimumElevatorHeight);
        elevator.setPower(0.4);

    }

    public void resetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    void stopmotors() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    void getColour(){
        sensorColor = hardwareMap.get(ColorSensor.class, "colourSensor");

        waitForStart();
        resetRuntime();

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
                location = 1;

            } else {
                telemetry.addLine("This is not green: Wrong Hue");
                //telemetry.update();
            }

            if (hsvValues[0] < 30 && hsvValues[0] > 10) {
                telemetry.addLine("This is red");
                location = 2;

            } else {
                telemetry.addLine("This is not red: Wrong Hue");
               // telemetry.update();
            }

            if (hsvValues[0] < 220 && hsvValues[0] > 190) {

                   telemetry.addLine("This is purple");
                    location = 3;

                } else {
                    telemetry.addLine("This is not purple: Wrong Hue");
                   // telemetry.update();

            }
            telemetry.addData("loco", location);
        telemetry.update();
        }


    void goLocation(double Speed, int location){

        if (location == 1){
            strafing(-1100, 0.5, false); //go left
            sleep(300);

        }
        else if (location == 2){
            sleep(300);
        }
        else if (location == 3){
            strafing(1100, 0.5, true); //go right

            sleep(300);
        }
        else {
            stopmotors();
        }
    }
}





