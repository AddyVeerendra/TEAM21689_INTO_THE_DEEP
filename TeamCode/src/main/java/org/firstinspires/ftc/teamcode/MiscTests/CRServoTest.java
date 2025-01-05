package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareClasses.ColorV3;

@TeleOp(name = "CRServo", group = "TeleOp")
public class CRServoTest extends LinearOpMode {

    // Declare the continuous rotation servo
    private CRServo continuousServo1;
    private Servo intakePitchServo;

    //private ColorV3 colorV3;

    // Time tracking variables for non-blocking delay
    private long reverseStartTime = 0;
    private boolean reversing = false;

    @Override
    public void runOpMode() {
        // Initialize the servo
        continuousServo1 = hardwareMap.get(CRServo.class, "intakePivot");
        intakePitchServo = hardwareMap.get(Servo.class, "intakeSlidesLeft");
        //colorV3 = new ColorV3(hardwareMap);

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
//            if (colorV3.isConnected()) {
//                if (reversing) {
//                    // If in reverse mode, check if 1 second has passed
//                    if (System.currentTimeMillis() - reverseStartTime >= 1000) {
//                        continuousServo1.setPower(-0.25);  // Resume intaking
//                        continuousServo2.setPower(0.25);
//                        reversing = false;  // Reset the reversing flag
//                    }
//                } else {
//                    // If proximity is within 1.5, stop intaking
//                    if (colorV3.proximity() < 1.5) {
//                        continuousServo1.setPower(0);  // Stop intake
//                        continuousServo2.setPower(0);
//                    } else {
//                        continuousServo1.setPower(-0.25);  // Continue intaking
//                        continuousServo2.setPower(0.25);
//                    }
//
//                    // If dpad_up is pressed, start reversing for 1 second
//                    if (gamepad1.dpad_up) {
//                        continuousServo1.setPower(0.25);  // Reverse intake
//                        continuousServo2.setPower(-0.25);
//                        reverseStartTime = System.currentTimeMillis();
//                        reversing = true;  // Set reversing flag
//                    }
//                }
//            } else {
//                requestOpModeStop();  // Stop the opmode if the color sesor is disconnected
//            }

            if (gamepad1.dpad_down) {
                continuousServo1.setPower(-1);
                intakePitchServo.setPosition(0.75);
            } else if (gamepad1.dpad_up) {
                continuousServo1.setPower(0);
                intakePitchServo.setPosition(0.4);
            }

            if (gamepad1.right_bumper) {
                intakePitchServo.setPosition(0.05);
            }
            telemetry.update();  // Update telemetry data
        }
    }
}