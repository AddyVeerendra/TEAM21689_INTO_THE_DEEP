package org.firstinspires.ftc.teamcode.LimeLight;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;


@TeleOp(name = "Megatag2RelocalizerTest", group = "Sensor")
@Disabled
public class Megatag2RelocalizerTest extends LinearOpMode {

    Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        Megatag2Relocalizer relocalizer = new Megatag2Relocalizer(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(12, -63.5, Math.toRadians(90)));

        telemetry.setMsTransmissionInterval(11);

        if (relocalizer.isLimelightConnected()) {
            telemetry.addData(">", "Limelight Connected");
        } else {
            telemetry.addData(">", "Limelight Not Connected");
        }
        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            follower.update();
            telemetry.addLine("Looking!");

            Pose3D botpose = relocalizer.getBotPose(follower.getPose().getHeading());
            if (botpose != null) {
                telemetry.addData("Limelight Pose", botpose.toString());
                telemetry.addData("Follower Pose", follower.getPose().toString());
                telemetry.addData("X", relocalizer.getXValue(botpose));
                telemetry.addData("Y", relocalizer.getYValue(botpose));
                telemetry.addData("Unit", relocalizer.getDistanceUnit(botpose));
            }
            telemetry.update();
        }
    }
}