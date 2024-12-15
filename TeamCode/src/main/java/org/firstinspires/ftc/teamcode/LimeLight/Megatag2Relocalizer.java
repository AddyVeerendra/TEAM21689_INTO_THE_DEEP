package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Megatag2Relocalizer {

    private final IMU imu;
    private final Limelight3A limelight;
    private static final double METERS_TO_INCHES = 39.3701;
    private static final double MM_TO_INCHES = 0.0393701;
    private static final double CM_TO_INCHES = 0.393701;

    public Megatag2Relocalizer(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        limelight.pipelineSwitch(7);
        limelight.start();
    }

    public boolean isLimelightConnected() {
        return limelight.isConnected();
    }

    public Pose3D getBotPose() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose_MT2();
            return convertPoseToInches(botpose);
        }
        return null;
    }

    public int getAprilTagID() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            int aprilTagId = result.getFiducialResults().get(0).getFiducialId();
            return aprilTagId;
        }
        return -1;
    }

    private Pose3D convertPoseToInches(Pose3D pose) {
        Position position = pose.getPosition();
        double conversionFactor;

        switch (position.unit) {
            case METER:
                conversionFactor = METERS_TO_INCHES;
                break;
            case CM:
                conversionFactor = CM_TO_INCHES;
                break;
            case MM:
                conversionFactor = MM_TO_INCHES;
                break;
            default:
                conversionFactor = 1.0;
        }

        double xInches = position.x * conversionFactor;
        double yInches = position.y * conversionFactor;
        double zInches = position.z * conversionFactor;

        return new Pose3D(new Position(DistanceUnit.INCH, xInches, yInches, zInches, position.acquisitionTime), pose.getOrientation());
    }
}