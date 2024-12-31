package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp
public class SampleDetectionTest extends LinearOpMode {
    private CameraManagerSample cameraManager;

    // Global variables for angle and servo position
    private double angle = 0.0;
    private double servoPosition = 0.0;

    @Override
    public void runOpMode() {
        cameraManager = new CameraManagerSample(hardwareMap);

        cameraManager.initializeCamera();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Retrieve detected samples from the pipeline
            List<SampleDetectionPipeline.Sample> detectedSamples = cameraManager.getDetectedSamples();

            if (!detectedSamples.isEmpty()) {
                for (SampleDetectionPipeline.Sample sample : detectedSamples) {
                    double x = sample.width;  // Width of bounding box in pixels
                    double y = sample.height; // Height of bounding box in pixels

                    // Process bounding box to calculate angle and servo position
                    processBoundingBox(x, y);

                    // Display all sample details on telemetry
                    telemetry.addData("Sample", "Color: %s, Area: %.2f, Center [X: %.2f, Y: %.2f]",
                            sample.color, sample.area, sample.centerX, sample.centerY);
                    telemetry.addData("Angle", "%.2f degrees", angle);
                    telemetry.addData("Servo Position", "%.3f", servoPosition);
                }
            } else {
                telemetry.addLine("No samples detected.");
            }

            telemetry.update();
            sleep(50);
        }
    }

    // Method to calculate angle and servo position
    private void processBoundingBox(double x, double y) {
        double minServoPosition = 0.0; // Minimum servo position
        double maxServoPosition = 0.3; // Maximum servo position
        double maxAngle = 90.0;        // Maximum angle

        // Calculate the angle as the arctangent of x / y
        angle = Math.toDegrees(Math.atan(x / y));

        // Normalize the angle to the range 0-90 degrees
        if (x > y) {
            angle = 90 - angle;
        }

        // Map the angle to the servo position range
        servoPosition = (angle / maxAngle) * (maxServoPosition - minServoPosition) + minServoPosition;
    }
}
