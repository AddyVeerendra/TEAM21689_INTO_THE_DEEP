package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.PriorityQueue;

@TeleOp(name = "PNP_TEST")
public class PNP_Test extends LinearOpMode {
    private CameraManagerPNP cameraManager;
    private static final float cameraAngle = 30;

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
                List<SamplePNP_Pipeline.Sample> samplesCopy = new ArrayList<>(detectedSamples);
                for (SamplePNP_Pipeline.Sample sample : samplesCopy) {
                    telemetry.addData("Color", sample.color);
                    telemetry.addData("Distance", sample.distance);
                    telemetry.addData("Extension Distance", getExtensionDistance(cameraAngle, sample.distance));
                    telemetry.addData("X Displacement", sample.displacementX);
                    telemetry.addData("Y Displacement", sample.displacementY);
                    telemetry.addData("Rotation", sample.rotation);
                    telemetry.addData("Rank", sample.rank);


                }
            }
            else {
                telemetry.addLine("No samples detected.");
            }

            telemetry.update();
            sleep(50);
        }
    }

    public double getExtensionDistance(double cameraAngle, double objectDistance){
        return (objectDistance * Math.sin(Math.toRadians(cameraAngle)));
    }
}