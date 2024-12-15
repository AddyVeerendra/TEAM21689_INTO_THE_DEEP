package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.LimeLight.Megatag2Relocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

import java.util.Arrays;
import java.util.List;


@TeleOp
public class LimelightRelocTest extends LinearOpMode {

    Megatag2Relocalizer relocalizer = new Megatag2Relocalizer(hardwareMap);

    private Follower follower;
    private Path forwards, backwards;
    private Timer pathTimer;
    private int pathState;

    private List<Integer> validAprilTags = Arrays.asList(12, 13, 14, 15, 16);
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setMsTransmissionInterval(11);

        waitForStart();

        while (opModeIsActive()) {
            //Pose3D botPose = relocalizer.getBotPose();
            Pose pose = follower.getPose();
            int aprilTagId = relocalizer.getAprilTagID();

            if (aprilTagId == -1) {
                telemetry.addData("April Tag ID", "No April Tag Detected");
            }

            else if (validAprilTags.contains(aprilTagId)) {
                Pose3D limelightPose = relocalizer.getBotPose();
                telemetry.addData("April Tag ID", aprilTagId);
                telemetry.addData("Bot Pose", pose);
                telemetry.addData("Limelight Pose", limelightPose);
            }
        }
        telemetry.update();
    }
}
