package org.firstinspires.ftc.teamcode;
//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name= "elevator test", group="Linear Opmode")


public class elevatortest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor elevator = null;

    @Override
    public void runOpMode() throws InterruptedException {

        elevator = hardwareMap.get(DcMotorEx.class, "elevator");
        elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setDirection(DcMotorEx.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                elevator.setPower(0.3);
                telemetry.addData("lift position", elevator.getCurrentPosition());
                telemetry.update();
            } else if (gamepad1.y) {
                elevator.setPower(-0.3);
                telemetry.addData("lift position", elevator.getCurrentPosition());
                telemetry.update();
            } else {
                elevator.setPower(0);
                telemetry.addData("lift position", elevator.getCurrentPosition());
                telemetry.update();
            }

        }
    }
}