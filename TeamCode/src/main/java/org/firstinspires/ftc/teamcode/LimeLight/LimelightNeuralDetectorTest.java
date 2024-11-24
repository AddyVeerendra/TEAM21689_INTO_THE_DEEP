package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HardwareClasses.Limelight;
import java.util.List;


@TeleOp(name = "LimelightDetectorTest", group = "Sensor")
public class LimelightNeuralDetectorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight limelight = new Limelight(hardwareMap);

        telemetry.setMsTransmissionInterval(11);
        limelight.getLimelight().setPollRateHz(11);
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (limelight.isConnected() && !limelight.getDetectorResults().isEmpty()) {
                limelight.getDetectorResults().forEach(dr -> {
                    if ("blue".equals(dr.getClassName())) {
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
                });
            } else {
                telemetry.addData("Status", "Limelight Not Connected");
            }

            telemetry.update();
        }
    }
}