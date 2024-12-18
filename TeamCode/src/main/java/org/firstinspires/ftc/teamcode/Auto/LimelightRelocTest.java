package org.firstinspires.ftc.teamcode.Auto;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.LimeLight.Megatag2Relocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.util.Arrays;
import java.util.List;

public class LimelightRelocTest {

    private Megatag2Relocalizer relocalizer;
    private Follower follower;
    private List<Integer> validAprilTags = Arrays.asList(12, 13, 14, 15, 16);

    public LimelightRelocTest(Megatag2Relocalizer relocalizer, Follower follower) {
        this.relocalizer = relocalizer;
        this.follower = follower;
    }

    public Pose relocalize() {
        Pose pose = follower.getPose();
        int aprilTagId = relocalizer.getAprilTagID();

        if (aprilTagId == -1) {
            return null; // No April Tag Detected
        } else if (validAprilTags.contains(aprilTagId)) {
            Pose3D limelightPose = relocalizer.getBotPose();
            if (limelightPose != null) {
                // Update the robot's pose using the Limelight's detected pose
                return new Pose(
                        limelightPose.getPosition().x,
                        limelightPose.getPosition().y,
                        limelightPose.getOrientation().getYaw()
                );
            }
        }
        return null; // No valid pose detected
    }
}