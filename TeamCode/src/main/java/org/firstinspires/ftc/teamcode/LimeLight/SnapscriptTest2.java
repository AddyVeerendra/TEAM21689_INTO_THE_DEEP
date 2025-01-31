package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp(name = "Snapscript Test")
@Disabled
public class SnapscriptTest2 extends LinearOpMode {
    private Limelight3A limelight;

    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.setPollRateHz(90);

        if (limelight.isConnected()) {
            telemetry.addData(">", "Limelight Connected");
        } else {
            telemetry.addData(">", "Limelight Not Connected");
        }

        telemetry.update();

        waitForStart();

        limelight.start();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            double[] pythonOutputs = result.getPythonOutput();

            if (pythonOutputs != null && pythonOutputs.length > 0) {
                telemetry.addData("Python output", Arrays.toString(pythonOutputs));
                telemetry.addData("Output", pythonOutputs);
                telemetry.addData("Status", limelight.isRunning());
                long staleness = result.getStaleness();
                telemetry.addData("Staleness", staleness);
            }

            telemetry.update();

            if(isStopRequested()){
                break;
            }
        }
        limelight.stop();

        telemetry.update();
    }
}
