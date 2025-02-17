package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.PriorityQueue;

public class CameraManagerPNP {
    private OpenCvWebcam webcam;
    private OldSamplePNP_Pipeline pipeline;

    public CameraManagerPNP(HardwareMap hardwareMap, Telemetry telemetry, double focalLength, double realObjectHeight) {
        // Get the camera monitor view ID for displaying the camera preview on the screen
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Create an instance of the webcam using the camera monitor view ID
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Create an instance of the SamplePNP_Pipeline
        pipeline = new OldSamplePNP_Pipeline(focalLength, realObjectHeight, telemetry);

        // Set the pipeline for the webcam
        webcam.setPipeline(pipeline);

        // Set a timeout for obtaining camera permission
        webcam.setMillisecondsPermissionTimeout(5000);
    }

    public void initializeCamera() {
        // Open the camera device asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming from the webcam with specified resolution and rotation
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT); // Ensure correct orientation
            }

            @Override
            public void onError(int errorCode) {
                // Handle any errors that occur while opening the camera
            }
        });
    }

    // Method to return all detected samples as a list of Sample objects
    public List<OldSamplePNP_Pipeline.Sample> getDetectedSamples() {
        return pipeline.getDetectedSamples();
    }

    public OldSamplePNP_Pipeline.Sample getHighestRankedSample() {
        return (pipeline.getDetectedSamples()).get(0);
    }
}