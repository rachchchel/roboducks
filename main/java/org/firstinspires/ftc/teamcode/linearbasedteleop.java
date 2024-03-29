

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Linear Teleop", group="Robot")

public class linearbasedteleop extends LinearOpMode {
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor  elevator = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;

    double clawOffset = 0;

    public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;
    public static final double ELEVATOR_UP_POWER    =  1 ;
    public static final double ELEVATOR_DOWN_POWER  = -1 ;

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        telemetry.addData("Angle:", angles.firstAngle);
        telemetry.update();

        double error = 0 - angles.firstAngle;
        double kp = 0.1;
        double p = error * kp * 0.3;

        leftFront.setPower(0.6 - p);
        leftBack.setPower(0.6 - p);
        rightFront.setPower(0.6 + p);
        rightBack.setPower(0.6 + p);

        waitForStart();
        double left;
        double right;
        double drive;
        double turn;



        // Define and Initialize Motors
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        elevator    = hardwareMap.get(DcMotor.class, "elevator");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);

        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "The robot is ready,, press play");    //
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {
            double max;
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double botHeading = -imu.getAngularOrientation().firstAngle;
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (rotY + rotX + rx) / denominator;
            double leftBackPower = (rotY - rotX + rx) / denominator;
            double rightFrontPower = (rotY - rotX - rx) / denominator;
            double rightBackPower = (rotY + rotX - rx) / denominator;

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.left_stick_x; //gamepad1.right_stick_x


            left  = drive + turn;
            right = drive - turn;

            max = Math.max(Math.abs(leftFrontPower), Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            leftFront.setPower(left);
            leftBack.setPower(left);
            rightFront.setPower(right);
            rightBack.setPower(right);

 //claw code
           if (gamepad1.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad1.left_bumper)
                clawOffset -= CLAW_SPEED;

            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            leftClaw.setPosition(MID_SERVO + clawOffset);
            rightClaw.setPosition(MID_SERVO - clawOffset);


//elevator code

            if (gamepad1.y) {
                elevator.setPower(ELEVATOR_UP_POWER);
            }
            else if (gamepad1.a) {
                elevator.setPower(ELEVATOR_DOWN_POWER);
            }
            else {
                elevator.setPower(0.0);
            }



            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

            sleep(50);
        }
    }
}
