package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Slide Control with Encoder", group = "Linear Opmode")
public class SimpleSlideControl extends LinearOpMode {

    // Declare the motor
    private DcMotor slideMotor;

    @Override
    public void runOpMode() {
        // Initialize the hardware
        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");

        // Set the motor's zero power behavior to BRAKE
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Reset the encoder and set the motor to run using the encoder
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set the motor power to zero at start
        slideMotor.setPower(0);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Check gamepad buttons and set motor target position
            if (gamepad1.x) {
                // Move motor to position 1100
                slideMotor.setTargetPosition(1100);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(1);  // Set power to move towards target
            } else if (gamepad1.y) {
                // Move motor to position 0
                slideMotor.setTargetPosition(0);
                slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotor.setPower(1);  // Set power to move towards target
            } else if (!slideMotor.isBusy()) {
                // Stop motor when not busy (reached target)
                slideMotor.setPower(0);
            }

            // Display the motor encoder position on telemetry
            telemetry.addData("Motor Power", slideMotor.getPower());
            telemetry.addData("Motor Target Position", slideMotor.getTargetPosition());
            telemetry.addData("Motor Encoder Position", slideMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}