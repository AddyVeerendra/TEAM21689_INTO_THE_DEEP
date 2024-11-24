package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp
public class SampleDetectionPNPTest extends LinearOpMode {
    // Declare variables for the webcam and the OpenCV pipeline
    private CameraManagerSamplePNP cameraManager;

    /**
     * The main method for the op mode. This method sets up and runs the OpenCV pipeline.
     */
    @Override
    public void runOpMode() {
        cameraManager = new CameraManagerSamplePNP(hardwareMap);

        // Initialize camera and set the pipeline to use SampleDetectionPipeline
        cameraManager.initializeCamera();

        // Notify that the op mode is waiting for the start signal
        telemetry.addLine("Waiting for start");
        telemetry.update();

        // Wait for the start signal
        waitForStart();

        // Run the op mode while it is active
        while (opModeIsActive()) {
            // Get the detected objects from the pipeline
            List<SampleDetectionPipelinePNP.AnalyzedStone> detectedStones = cameraManager.pipelinePNP().getDetectedStones();

            // Display the number of detected objects
            telemetry.addData("Detected Stones", detectedStones.size());

            // Loop through each detected stone and display its data
            for (SampleDetectionPipelinePNP.AnalyzedStone stone : detectedStones) {
                telemetry.addData("Color", stone.color);
                telemetry.addData("Angle", stone.angle);
                telemetry.addData("Translation (x, y, z)",
                        "%.2f, %.2f, %.2f",
                        stone.tvec.get(0, 0)[0],
                        stone.tvec.get(1, 0)[0],
                        stone.tvec.get(2, 0)[0]
                );
            }

            // Update the telemetry display
            telemetry.update();

            // Sleep briefly to avoid overwhelming the system
            sleep(100);
        }
    }
}