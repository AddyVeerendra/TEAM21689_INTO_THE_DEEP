package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareClasses.ColorV3;

@TeleOp(name = "Intake Slides Test", group = "TeleOp")
public class IntakeSlidesTest extends LinearOpMode {

    // Declare the continuous rotation servo
    private Servo intakeSlideLeft;
    private Servo intakeSlideRight;

    @Override
    public void runOpMode() {
        // Initialize the servo
        intakeSlideLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeSlideLeft.setDirection(Servo.Direction.REVERSE);
        intakeSlideRight = hardwareMap.get(Servo.class, "intakeServoRight");

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                intakeSlideLeft.setPosition(0);
                intakeSlideRight.setPosition(0);
            } else if (gamepad1.dpad_right) {
                intakeSlideLeft.setPosition(0.3);
                intakeSlideRight.setPosition(0.3);
            }
        }
    }
}