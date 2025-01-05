package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LimeLight.ColorSampleDisplacement;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "RedSampleAlignTest")
public class RedSampleAlignTest extends OpMode {
    private ColorSampleDisplacement colorSampleDisplacement;
    private Follower follower;

    @Override
    public void init() {
        colorSampleDisplacement = new ColorSampleDisplacement(hardwareMap, "blue");
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(40, 40, Math.toRadians(15)));
        telemetry.setMsTransmissionInterval(11);
    }

    @Override
    public void loop() {
        follower.update();
        double displacement = colorSampleDisplacement.getDisplacementInPixels();
        if (!Double.isNaN(displacement)) {
            if (displacement < -2) {
                telemetry.addLine("Sample not aligned");
                telemetry.addData("Displacement", displacement);
                follower.startTeleopDrive();
                follower.setTeleOpMovementVectors(0, 0.5, 0); // Adjust the speed as needed
            } else if (displacement > 2) {
                telemetry.addLine("Sample not aligned");
                telemetry.addData("Displacement", displacement);
                follower.startTeleopDrive();
                follower.setTeleOpMovementVectors(0, -0.5, 0); // Adjust the speed as needed
            } else {
                telemetry.addLine("Sample aligned");
                follower.breakFollowing();
            }

        } else {
            telemetry.addLine("Sample not detected");
        }
        telemetry.update();
    }
}