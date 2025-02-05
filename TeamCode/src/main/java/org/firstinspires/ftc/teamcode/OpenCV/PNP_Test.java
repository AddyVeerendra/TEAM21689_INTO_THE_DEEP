// SomeOtherOpMode.java
package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Use_PNP_Data")
public class PNP_Test extends LinearOpMode {

    private CameraManagerPNP cameraManager;

    @Override
    public void runOpMode() {
        // Initialize the camera manager (assumes proper constructor and initialization in your project)
        cameraManager = new CameraManagerPNP(hardwareMap, telemetry, 822.317, 8.9);
        cameraManager.initializeCamera();

        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Extract the PnP data using our helper class.
            PNPDataExtractor.SampleData sampleData = PNPDataExtractor.extractPNPData(cameraManager);

            if (sampleData != null) {
                telemetry.addData("Color", sampleData.color);
                telemetry.addData("Horizontal Displacement (cm)", sampleData.horizontalDisplacement);
                telemetry.addData("Vertical Displacement (cm)", sampleData.verticalDisplacement);
                telemetry.addData("Rank", sampleData.rank);
                telemetry.addData("Is Horizontal", sampleData.isHorizontal);
            } else {
                telemetry.addLine("No samples detected.");
            }

            telemetry.update();
            sleep(50);
        }
    }
}
