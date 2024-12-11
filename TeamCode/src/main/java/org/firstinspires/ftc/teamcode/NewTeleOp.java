package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeAssembly;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;

@TeleOp (name = "NEWEST V2 Robot TeleOp")
public class NewTeleOp extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private IntakeAssembly intakeAssembly;
    private LinearSlide linearSlides;

    // Toggle states
    private boolean clawOpen = false;
    private boolean clawRotated = false;
    private boolean sequenceToggle = false;

    // Track button states
    private boolean rightBumperPressed = false;
    private boolean yPressed = false;
    private boolean aPressed = false;

    @Override
    public void runOpMode() {

        // Initialize mecanum drive motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set motor directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        String[] motorNames = {"slideMotorLeft", "slideMotorRight"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.REVERSE, DcMotorSimple.Direction.FORWARD};
        LinearSlide linearSlide = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 37.5); // Example ticksPerInch and limits

        while (opModeInInit() && !isStopRequested()) {

        }

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, check proximity and detect color
        while (opModeIsActive()) {
            // Mecanum drive control
            double y = -gamepad1.left_stick_y; // Invert Y axis
            double x = gamepad1.left_stick_x * 1.1; // Adjust for strafing power
            double rx = gamepad1.right_stick_x;

            // Calculate motor powers
            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            // Clip the motor powers to ensure they are within the range [-1, 1]
            frontLeftPower = Math.max(-1, Math.min(1, frontLeftPower));
            backLeftPower = Math.max(-1, Math.min(1, backLeftPower));
            frontRightPower = Math.max(-1, Math.min(1, frontRightPower));
            backRightPower = Math.max(-1, Math.min(1, backRightPower));

            // Set the motor powers
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            // Right bumper toggles claw open/closed
            if (gamepad1.right_bumper && !rightBumperPressed) {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    intakeAssembly.CloseClaw();
                } else {
                    intakeAssembly.OpenClaw();
                }
                rightBumperPressed = true;
            } else if (!gamepad1.right_bumper) {
                rightBumperPressed = false;
            }

            // Left bumper toggles claw rotation
            if (gamepad1.y && !yPressed) {
                clawRotated = !clawRotated;
                if (clawOpen) {
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
                sequenceToggle = !sequenceToggle;
                if (sequenceToggle) {
                    // Sequence 1
                    intakeAssembly.CloseClaw();
                    sleep(150);
                    intakeAssembly.RotateClaw0();
                    intakeAssembly.RetractSlidesFull();
                    intakeAssembly.PivotClawUp();
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
        }
    }
}