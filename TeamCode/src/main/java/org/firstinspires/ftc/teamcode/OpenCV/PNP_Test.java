package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "PNP_TEST")
public class PNP_Test extends LinearOpMode {
    private CameraManagerPNP cameraManager;

    @Override
    public void runOpMode() {
        cameraManager = new CameraManagerPNP(hardwareMap, telemetry, 822.317, 8.9);

        cameraManager.initializeCamera();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Retrieve detected samples from the pipeline
            List<SamplePNP_Pipeline.Sample> detectedSamples = cameraManager.getDetectedSamples();

            if (!detectedSamples.isEmpty()) {
                for (SamplePNP_Pipeline.Sample sample : detectedSamples) {

                    // Display all sample details on telemetry
                    telemetry.addData("Color", sample.color);
                    telemetry.addData("Distance", sample.distance);
                }
            } else {
                telemetry.addLine("No samples detected.");
            }

            telemetry.update();
            sleep(50);
        }
    }
}
