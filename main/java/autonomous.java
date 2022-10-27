import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Autonomous", group="anything")

public class AutoDrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor driveOne = null;
    public DcMotor driveTwo = null;
    public DcMotor driveThree = null;
    public DcMotor driveFour = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;

    double clawOffset  = 0;

    public static final double MID_SERVO = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
            driveOne = hardwareMap.get(DcMotor.class, "drive_one");
            driveTwo = hardwareMap.get(DcMotor.class, "drive_two");
            driveThree = hardwareMap.get(DcMotor.class, "drive_three");
            driveFour = hardwareMap.get(DcMotor.class, "drive_four");

            driveOne.setDirection(DcMotor.Direction.FORWARD);
            driveTwo.setDirection(DcMotor.Direction.REVERSE);
            driveThree.setDirection(DcMotor.Direction.FORWARD);
            driveFour.setDirection(DcMotor.Direction.REVERSE);

            leftClaw = hardwareMap.get(Servo.class, "left_hand");
            rightClaw = hardwareMap.get(Servo.class, "right_hand");
            leftClaw.setPosition(MID_SERVO);
            rightClaw.setPosition(MID_SERVO);

            waitForStart();
            if (opModeIsActive()) {
                rightTarget = RightDrive.getCurrentPosition() + (int)(30 * DRIVE_COUNTS_PER_IN);
                leftTarget = LeftDrive.getCurrentPosition() + (int)(15 * DRIVE_COUNTS_PER_IN);
            }

        runtime.reset();

    }
}


