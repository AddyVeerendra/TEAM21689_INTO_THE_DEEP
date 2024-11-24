package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp
public class SampleDetectionTest extends LinearOpMode {
    // Declare variables for the webcam and the OpenCV pipeline
    private CameraManagerSample cameraManager;

    /**
     * The main method for the op mode. This method sets up and runs the OpenCV pipeline.
     */
    @Override
    public void runOpMode() {
        cameraManager = new CameraManagerSample(hardwareMap);

        // Initialize camera and set the pipeline to use SampleDetectionPipeline
        cameraManager.initializeCamera();

        // Notify that the op mode is waiting for the start signal
        telemetry.addLine("Waiting for start");
        telemetry.update();

        // Wait for the start signal
        waitForStart();

        // Run the op mode while it is active
        while (opModeIsActive()) {
            // Get the list of all detected samples (Sample objects) from the pipeline
            List<SampleDetectionPipeline.Sample> detectedSamples = cameraManager.getDetectedSamples();

            if (!detectedSamples.isEmpty()) {
                // Loop through all detected samples and display their information on the telemetry
                for (SampleDetectionPipeline.Sample sample : detectedSamples) {
                    telemetry.addData("Sample", "Color: %s, Area: %.2f, Center [X: %.2f, Y: %.2f]",
                            sample.color, sample.area, sample.centerX, sample.centerY);
                }
            } else {
                telemetry.addLine("No samples detected.");
            }

            // Update the telemetry
            telemetry.update();

            // Sleep for a short duration to avoid excessive processing
            sleep(50);
        }
    }
}