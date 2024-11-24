package org.firstinspires.ftc.teamcode.pedroPathing.localization.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Independent Encoder Telemetry", group = "Autonomous Pathing Tuning")
public class IndependentEncoderTelemetry extends LinearOpMode {

    private DcMotorEx leftEncoder;
    private DcMotorEx rightEncoder;
    private DcMotorEx strafeEncoder;

    @Override
    public void runOpMode() {
        // Initialize the encoders
        leftEncoder = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "leftFront");
        strafeEncoder = hardwareMap.get(DcMotorEx.class, "leftBack");

        // Reverse encoder directions if needed
        leftEncoder.setDirection(DcMotorEx.Direction.FORWARD);
        rightEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        strafeEncoder.setDirection(DcMotorEx.Direction.FORWARD);

        telemetry.addLine("Encoders initialized. Ready to start.");
        telemetry.update();

        // Wait for the start signal
        waitForStart();

        // Main loop to display encoder values
        while (opModeIsActive()) {
            // Read encoder positions
            double leftPosition = leftEncoder.getCurrentPosition();
            double rightPosition = rightEncoder.getCurrentPosition();
            double strafePosition = strafeEncoder.getCurrentPosition();

            // Display the positions on telemetry
            telemetry.addData("Left Encoder Position", leftPosition);
            telemetry.addData("Right Encoder Position", rightPosition);
            telemetry.addData("Strafe Encoder Position", strafePosition);
            telemetry.update();
        }
    }
}