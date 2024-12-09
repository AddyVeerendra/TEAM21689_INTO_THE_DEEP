package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class ColorSampleDisplacement {

    private Limelight3A limelight;
    private String targetColor;

    public ColorSampleDisplacement(HardwareMap hardwareMap, String targetColor) {
        this.limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        this.targetColor = targetColor;
        this.limelight.pipelineSwitch(1);
        this.limelight.start();
    }

    public boolean isConnected() {
        return limelight.isConnected();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public double getDisplacementInPixels() {
        LLResult result = getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            for (LLResultTypes.DetectorResult dr : detectorResults) {
                if (dr.getClassName().equals(targetColor)) {
                    return dr.getTargetXPixels();
                }
            }
        }
        return Double.NaN;
    }

    public Pose3D getBotPose() {
        LLResult result = getLatestResult();
        if (result != null) {
            return result.getBotpose();
        }
        return null;
    }

    public double getLatency() {
        LLResult result = getLatestResult();
        if (result != null) {
            return result.getCaptureLatency() + result.getTargetingLatency();
        }
        return Double.NaN;
    }

    public double getParseLatency() {
        LLResult result = getLatestResult();
        if (result != null) {
            return result.getParseLatency();
        }
        return Double.NaN;
    }
}