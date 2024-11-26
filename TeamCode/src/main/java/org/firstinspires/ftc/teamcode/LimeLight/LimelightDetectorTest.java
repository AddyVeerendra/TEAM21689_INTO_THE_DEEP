package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "LimelightDetectorTest", group = "Sensor")
public class LimelightDetectorTest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {


        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(1);

        limelight.start();

        if (limelight.isConnected()) {
            telemetry.addData(">", "Limelight Connected");
        } else {
            telemetry.addData(">", "Limelight Not Connected");
        }
        telemetry.addData(">", "Robot Ready.  Press Play.");

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            telemetry.addLine("Looking");

            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());

                    // Access barcode results
                    List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
                    for (LLResultTypes.DetectorResult dr : detectorResults) {
                        if (dr.getClassName().equals("red")) {
                            List<List<Double>> targetCorners = dr.getTargetCorners();

                            double diagonalPixels = DistanceEstimator.getObjectDiagonalInPixels(targetCorners);
                            double objectWidthPixels = DistanceEstimator.getObjectWidthInPixels(targetCorners);
                            double objectHeightPixels = DistanceEstimator.getObjectHeightInPixels(targetCorners);

                            // Calculate the aspect ratio
                            double aspectRatio = objectWidthPixels / objectHeightPixels;
                            double distanceEstimate;

                            // Automatic method selection based on aspect ratio
                            if (aspectRatio > 1.2) {
                                distanceEstimate = DistanceEstimator.estimateDistanceFromWidth(DistanceEstimator.REAL_WORLD_WIDTH, objectWidthPixels);
                                telemetry.addData("Method Used", "Width Distance");
                            } else if (aspectRatio < 0.8) {
                                distanceEstimate = DistanceEstimator.estimateDistanceFromHeight(DistanceEstimator.REAL_WORLD_HEIGHT, objectHeightPixels);
                                telemetry.addData("Method Used", "Height Distance");
                            } else {
                                distanceEstimate = DistanceEstimator.estimateDistanceFromDiagonal(
                                        Math.sqrt(Math.pow(DistanceEstimator.REAL_WORLD_WIDTH, 2) + Math.pow(DistanceEstimator.REAL_WORLD_HEIGHT, 2)),
                                        diagonalPixels
                                );
                                telemetry.addData("Method Used", "Diagonal Distance");
                            }

                            telemetry.addData("Estimated Distance", "%.2f meters", distanceEstimate);
                        }
                        telemetry.addData("Detector", "Class: %s, Area: %.2f", dr.getClassName(), dr.getTargetArea());
                    }
                }
            }

            telemetry.update();
        }
    }
}