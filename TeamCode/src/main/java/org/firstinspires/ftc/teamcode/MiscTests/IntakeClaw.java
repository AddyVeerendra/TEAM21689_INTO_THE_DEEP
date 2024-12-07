package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Intake Claw Test", group = "TeleOp")
public class IntakeClaw extends LinearOpMode {

    // Declare the continuous rotation servo
    private Servo intakePivot;
    private Servo intakeRotate;
    private Servo intakeClaw;
    private Servo intakeSlidesLeft;


    @Override
    public void runOpMode() {
        // Initialize the servo
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        intakePivot.setDirection(Servo.Direction.REVERSE);
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeSlidesLeft = hardwareMap.get(Servo.class, "intakeSlidesLeft");

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                intakeRotate.setPosition(0);
            } else if (gamepad1.dpad_right) {
                intakeRotate.setPosition(0.3);
            }

            if (gamepad1.left_bumper) {
                intakeClaw.setPosition(1);
            } else if (gamepad1.right_bumper) {
                intakeClaw.setPosition(0.75);
            }

            if (gamepad1.dpad_up) {
                intakePivot.setPosition(0);
            } else if (gamepad1.dpad_down){
                intakePivot.setPosition(0.75);
            }

            if (gamepad1.left_trigger > 0.5) {
                intakeSlidesLeft.setPosition(0);
            } else if (gamepad1.right_trigger > 0.5) {
                intakeSlidesLeft.setPosition(0.27);
            }
        }
    }
}