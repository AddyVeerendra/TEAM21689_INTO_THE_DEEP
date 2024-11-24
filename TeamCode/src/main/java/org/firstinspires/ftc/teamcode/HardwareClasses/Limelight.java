package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Limelight {

    private final Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap) {
        // Initialize the Limelight object using the HardwareMap
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    // Start the Limelight
    public void start() {
        limelight.start();
    }

    // Stop the Limelight
    public void stop() {
        limelight.stop();
    }

    // Get the Limelight's latest horizontal angle (tx)
    public double getTx() {
        return limelight.getLatestResult().getTx();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }
    // Get the pose of the robot (botpose) from the latest result
    public Pose3D getBotpose() {
        return limelight.getLatestResult().getBotpose_MT2();
    }

    // Get detector results (e.g., for object detection)
    public List<LLResultTypes.DetectorResult> getDetectorResults() {
        return limelight.getLatestResult().getDetectorResults();
    }

    public void pipelineSwitch(int i) {
        limelight.pipelineSwitch(i);
    }

    public boolean isConnected() {
        return limelight.isConnected();
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    public void updateRobotOrientation(double robotYaw) {
        limelight.updateRobotOrientation(robotYaw);
    }


}
