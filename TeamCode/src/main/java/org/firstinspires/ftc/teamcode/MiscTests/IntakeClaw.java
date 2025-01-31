package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Intake Claw Test", group = "TeleOp")
@Disabled
public class IntakeClaw extends LinearOpMode {

    // Declare the servos
    private Servo intakePivot;
    private Servo intakeRotate;
    private Servo intakeClaw;
    private Servo intakeSlidesLeft;

    // Toggle states
    private boolean clawOpen = false;
    private boolean clawRotated = false;
    private boolean sequenceToggle = false;

    // Track button states
    private boolean rightBumperPressed = false;
    private boolean leftBumperPressed = false;
    private boolean dpadRightPressed = false;

    @Override
    public void runOpMode() {
        // Initialize the servos
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        intakePivot.setDirection(Servo.Direction.REVERSE);
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeSlidesLeft = hardwareMap.get(Servo.class, "intakeSlidesLeft");

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // Right bumper toggles claw open/closed
            if (gamepad1.right_bumper && !rightBumperPressed) {
                clawOpen = !clawOpen;
                intakeClaw.setPosition(clawOpen ? 1.0 : 0.75);
                rightBumperPressed = true;
            } else if (!gamepad1.right_bumper) {
                rightBumperPressed = false;
            }

            // Left bumper toggles claw rotation
            if (gamepad1.left_bumper && !leftBumperPressed) {
                clawRotated = !clawRotated;
                intakeRotate.setPosition(clawRotated ? 0.3 : 0.0);
                leftBumperPressed = true;
            } else if (!gamepad1.left_bumper) {
                leftBumperPressed = false;
            }

            // D-pad right toggles between sequences
            if (gamepad1.dpad_right && !dpadRightPressed) {
                sequenceToggle = !sequenceToggle;
                if (sequenceToggle) {
                    // Sequence 1
                    intakeClaw.setPosition(1.0);
                    sleep(150);
                    intakeRotate.setPosition(0);
                    intakeSlidesLeft.setPosition(0.3);
                    intakePivot.setPosition(0.65);
                } else {
                    // Sequence 2
                    intakeSlidesLeft.setPosition(0.7);
                    intakePivot.setPosition(0.0);
                    intakeClaw.setPosition(0.75);
                }
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }
        }
    }
}