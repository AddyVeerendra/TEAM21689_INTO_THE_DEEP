package org.firstinspires.ftc.teamcode.LimeLight;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
// import telemetry here

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class Snapscript {
    private  Limelight3A limelight = null;
    private  ElapsedTime timer = null;
    private static final double MIN_UPDATE_INTERVAL = 0.05; // 50ms
    private Telemetry telemetry;
    /**
     * Constructor for the Snapscript class.
     * @param hardwareMap The HardwareMap from the OpMode
     */
    public Snapscript(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            this.telemetry  = telemetry;
            telemetry.addLine("Limelight initialized");
            telemetry.update();
            sleep(3000);
            limelight = hardwareMap.get(Limelight3A.class, "Limelight");

            telemetry.setMsTransmissionInterval(11);

            limelight.pipelineSwitch(0);

            limelight.setPollRateHz(90);

            timer = new ElapsedTime();
            limelight.start();

        } catch (Exception e) {
            System.err.println("Error initializing Snapscript: " + e.getMessage());
            //setup telemetry object here to print error message
            telemetry.addData("Error", "Error initializing Snapscript: " + e.getMessage());
            telemetry.update();
        }
    }

    /**
     * Retrieves the Python output from the Limelight.
     * @return An array of doubles containing the Python output, or null if no valid data is available.
     */
    static int ctr = 0;
    public double[] getPython() {
        try {
            if (timer.seconds() < MIN_UPDATE_INTERVAL) {
                sleep((long)((MIN_UPDATE_INTERVAL - timer.seconds()) * 1000));
            }
            timer.reset();

            telemetry.addLine("Getting Result");
            telemetry.update();
            sleep(3000);
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                double[] pythonOutputs = result.getPythonOutput();
                if (pythonOutputs != null && pythonOutputs.length > 0) {
                    System.out.println("Python output: " + ctr +  Arrays.toString(pythonOutputs));
                    telemetry.addData("Python output" + ctr, Arrays.toString(pythonOutputs));
                    ctr++;

                    long staleness = result.getStaleness();
                    telemetry.addData("Staleness", staleness);
                    telemetry.update();

                    return pythonOutputs;
                } else {
                    System.out.println("Python output is empty or null");
                    telemetry.addLine("Empty output");
                    telemetry.update();

                }
            } else {
                System.out.println("Limelight result is null");
                telemetry.addLine("Null output");
                telemetry.update();
            }
        } catch (Exception e) {
            System.err.println("Error getting Python output: " + e.getMessage());
            telemetry.addData("Error getting Python output", e.getMessage());
            e.printStackTrace();
            telemetry.update();
        }
        return null;
    }

    /**
     * Checks if the Limelight is connected.
     * @return true if the Limelight is connected, false otherwise.
     */
    public boolean isLimelightConnected() {
        return limelight.isConnected();
    }
}