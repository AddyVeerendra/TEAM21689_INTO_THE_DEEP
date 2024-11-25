package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "LimelightConfigTest", group = "Sensor")
public class LimelightConfigTest extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {


        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(7);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
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

            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();

            if (result != null) {
                // Access general information
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    //telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    //telemetry.addData("tync", result.getTyNC());

                    telemetry.addData("Botpose", botpose.toString());

                    // Access barcode results
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fr : fiducialResults) {
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }
                }
            }

            telemetry.update();
        }
    }
}