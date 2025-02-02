// PNP_Test.java
package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeAssemblyClaw;
import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeAssemblyIntake;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;

import java.util.List;

@TeleOp(name = "PNP_TEST")
public class PNP_Test extends LinearOpMode {
    private CameraManagerPNP cameraManager;
    private LinearSlide linearSlide;
    private static final float cameraAngle = 30;
    private IntakeAssemblyClaw intakeAssembly;

    @Override
    public void runOpMode() {
        cameraManager = new CameraManagerPNP(hardwareMap, telemetry, 822.317, 8.9);
        intakeAssembly = new IntakeAssemblyClaw(hardwareMap);
        cameraManager.initializeCamera();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Retrieve detected samples from the pipeline
            List<SamplePNP_Pipeline.Sample> detectedSamples = cameraManager.getDetectedSamples();

            if (!detectedSamples.isEmpty()) {
                SamplePNP_Pipeline.Sample highestRankedSample = cameraManager.getHighestRankedSample();
                if (highestRankedSample != null) {
                    double distanceCm = getExtensionDistance(15, highestRankedSample.distance);
                    double distanceInches = distanceCm / 2.54; // Convert centimeters to inches

                    intakeAssembly.ExtendSlidesToPos(distanceInches*74/32 + 2);

                    telemetry.addData("Color", highestRankedSample.color);
                    telemetry.addData("Distance (cm)", distanceCm);
                    telemetry.addData("Distance (inches)", distanceInches);
                    telemetry.addData("Extension Distance", getExtensionDistance(cameraAngle, distanceCm));
                    telemetry.addData("X Displacement", highestRankedSample.displacementX);
                    telemetry.addData("Y Displacement", highestRankedSample.displacementY);
                    telemetry.addData("Rotation", highestRankedSample.rotation);
                    telemetry.addData("Rank", highestRankedSample.rank);
                }
            } else {
                telemetry.addLine("No samples detected.");
            }

            telemetry.update();
            sleep(50);
        }
    }

    public double getExtensionDistance(double cameraAngle, double objectDistance) {
        return (objectDistance * Math.sin(Math.toRadians(cameraAngle)));
    }
}