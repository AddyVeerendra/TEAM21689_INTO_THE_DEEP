package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;


@TeleOp(name = "Snapscript", group = "Sensor")
public class SnapscriptTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        Snapscript snapscript = new Snapscript(hardwareMap, telemetry);

        telemetry.update();

        sleep(3000);

        if (snapscript.isLimelightConnected()) {
            telemetry.addData(">", "Limelight Connected");
        } else {
            telemetry.addData(">", "Limelight Not Connected");
        }
        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double[] pythonOutput = snapscript.getPython();
            if (pythonOutput != null) {
                String outputString = Arrays.toString(pythonOutput);
                telemetry.addData("Python Output", outputString);
                telemetry.addData("Output", pythonOutput);
            } else {
                telemetry.addData("Python Output", "No valid data");
            }
            telemetry.update();

            // Add a small delay to prevent overwhelming the telemetry
            sleep(500);
        }
    }
}