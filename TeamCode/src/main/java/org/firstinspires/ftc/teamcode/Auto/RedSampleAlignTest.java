package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LimeLight.ColorSampleDisplacement;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@TeleOp(name="RedSampleAlignTest")
public class RedSampleAlignTest extends OpMode {
    private ColorSampleDisplacement colorSampleDisplacement;
    private Follower follower;

    @Override
    public void init() {
        colorSampleDisplacement = new ColorSampleDisplacement(hardwareMap, "blue");
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(40, 40, Math.toRadians(15)));
        telemetry.setMsTransmissionInterval(11);
    }

    @Override
    public void loop() {
        double displacement = colorSampleDisplacement.getDisplacementInPixels();
        if (!Double.isNaN(displacement)) {
            if (Math.abs(displacement) < 2) {
                telemetry.addLine("Sample aligned");
            } else {
                telemetry.addLine("Sample not aligned");
                telemetry.addData("Displacement", displacement);
                Pose currentPose = follower.getPose();
                while (Math.abs(displacement) >= 2) {
                    if (displacement > 0) {
                        Pose newPose = new Pose(currentPose.getX(), currentPose.getY() - 1, currentPose.getHeading());
                        follower.followPath(new Path(new BezierLine(new Point(currentPose), new Point(newPose))));
                    } else if (displacement < 0) {
                        Pose newPose = new Pose(currentPose.getX(), currentPose.getY() + 1, currentPose.getHeading());
                        follower.followPath(new Path(new BezierLine(new Point(currentPose), new Point(newPose))));
                    }
                    displacement = colorSampleDisplacement.getDisplacementInPixels();
                    telemetry.addData("Displacement", displacement);
                    telemetry.update();
                }
            }
        } else {
            telemetry.addLine("No valid target detected");
        }
        telemetry.update();
    }
}