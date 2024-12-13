package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareClasses.DepositAssembly;
import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeAssembly;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;

@TeleOp (name = "NEWEST V2 Robot TeleOp")
public class NewTeleOp extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private IntakeAssembly intakeAssembly;
    private DepositAssembly depositAssembly;
    private LinearSlide linearSlides;

    // Toggle states
    private boolean clawOpen = false;
    private boolean clawRotated = true;
    private boolean intakeSequenceToggle = false;
    private boolean depositSampleToggle = false;

    // Track button states
    private boolean rightBumperPressed = false;
    private boolean yPressed = false;
    private boolean aPressed = false;
    private boolean dpadLeftPressed = false;

    @Override
    public void runOpMode() {

        // Initialize mecanum drive motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        String[] motorNames = {"slideMotorLeft", "slideMotorRight"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE};
        linearSlides = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 37.5); // Example ticksPerInch and limits

        intakeAssembly = new IntakeAssembly(hardwareMap);

        depositAssembly = new DepositAssembly(hardwareMap);

        while (opModeInInit() && !isStopRequested()) {

        }

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, check proximity and detect color
        while (opModeIsActive()) {
            // Calculate speed multiplier based on left trigger
            double speedMultiplier = 1 - (0.7 * gamepad2.left_trigger); // Map 0 to 1 (full speed) to 1 to 0.2

            // Mecanum drive control
            double y = -gamepad2.left_stick_y * speedMultiplier; // Forward/backward
            double x = gamepad2.left_stick_x * 1.1 * speedMultiplier; // Strafing
            double rx = gamepad2.right_stick_x * speedMultiplier; // Rotation

            // Calculate motor powers
            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            // Clip motor powers to ensure they're within [-1, 1]
            frontLeftPower = Math.max(-1, Math.min(1, frontLeftPower));
            backLeftPower = Math.max(-1, Math.min(1, backLeftPower));
            frontRightPower = Math.max(-1, Math.min(1, frontRightPower));
            backRightPower = Math.max(-1, Math.min(1, backRightPower));

            // Set motor powers
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

//            // Right bumper toggles claw open/closed
//            if (gamepad1.right_bumper && !rightBumperPressed) { // CHANGE TO OUTTAKE
//                clawOpen = !clawOpen;
//                if (clawOpen) {
//                    depositAssembly.OpenOuttakeClaw();
//                } else {
//                    depositAssembly.CloseOuttakeClaw();
//                    sleep(150);
//                    intakeAssembly.OpenClaw();
//                }
//                rightBumperPressed = true;
//            } else if (!gamepad1.right_bumper) {
//                rightBumperPressed = false;
//            }

            // Left bumper toggles claw rotation
            if (gamepad1.y && !yPressed) {
                clawRotated = !clawRotated;
                if (clawRotated) {
                    intakeAssembly.RotateClaw0();
                } else {
                    intakeAssembly.RotateClaw90();
                }
                yPressed = true;
            } else if (!gamepad1.y) {
                yPressed = false;
            }

            if (gamepad1.x) {
                intakeAssembly.PivotClawDown();
            }

            // D-pad right toggles between sequences
            if (gamepad1.a && !aPressed) {
                intakeSequenceToggle = !intakeSequenceToggle;
                if (intakeSequenceToggle) {
                    // Sequence 1
                    intakeAssembly.CloseClaw();
                    depositAssembly.OpenOuttakeClaw();
                    sleep(150);
                    intakeAssembly.RotateClaw0();
                    intakeAssembly.PivotClawUp();
                    sleep(400);
                    intakeAssembly.ExtendSlidesToPos(0.39);
                    sleep(600);
                    depositAssembly.CloseOuttakeClaw();
                    sleep(150);
                    intakeAssembly.OpenClaw();
                } else {
                    // Sequence 2
                    intakeAssembly.ExtendSlidesFull();
                    intakeAssembly.PivotClawMid();
                    intakeAssembly.OpenClaw();
                }
                aPressed = true;
            } else if (!gamepad1.a) {
                aPressed = false;
            }

            if (gamepad1.dpad_left && !dpadLeftPressed) {
                depositSampleToggle = !depositSampleToggle;
                if (depositSampleToggle) {
                    // Sequence 1
                    depositAssembly.OpenOuttakeClaw();
                    sleep(150);
                    linearSlides.moveSlidesToPositionInches(0);
                    depositAssembly.TransferSample();
                } else {
                    // Sequence 2
                    linearSlides.moveSlidesToPositionInches(35);
                    depositAssembly.ScoreSample();
                }
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }

            linearSlides.update();
        }
    }
}