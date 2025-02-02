package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "TeleOp")
@Disabled
public class TeleOpDrive extends OpMode {
    private Follower follower;

    // Variables for auto-align functionality
    private boolean isAutoAligning = false; // Track if the robot is in auto-align mode
    private boolean isAlignButtonPressed = false; // Prevent multiple align triggers

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        if (isAutoAligning) {
            // Check if the robot has finished aligning
            if (!follower.isBusy() || gamepad1.right_bumper) {
                isAutoAligning = false; // Alignment is complete
                follower.startTeleopDrive(); // Resume teleop driving
            }
        } else {
            // Normal teleop driving when not aligning
            updateDrive();

            // Trigger auto-align when 'A' button is pressed
            if (gamepad1.a && !isAlignButtonPressed) {
                isAlignButtonPressed = true; // Prevent multiple triggers
                initiateAutoAlign(); // Start the auto-align process
            } else if (!gamepad1.a) {
                isAlignButtonPressed = false; // Reset button press state
            }
        }
        follower.update();
    }

    public void updateDrive() {
        // --- Speed Changer Logic ---
        boolean slowerDriving = gamepad1.right_bumper;
        double stickScale = slowerDriving ? 0.4 : 1.0;
        double slowerTurning = 0.8;

        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y * stickScale,
                -gamepad1.left_stick_x * stickScale,
                -gamepad1.right_stick_x * stickScale * slowerTurning
        );
    }

    private void initiateAutoAlign() {
        // Build a simple path to the alignment point (you can modify this as needed)
        Path alignmentPath = new Path(new BezierLine(
                new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                new Point(0, 0, Point.CARTESIAN)));
        alignmentPath.setLinearHeadingInterpolation(follower.getPose().getHeading(), Math.toRadians(0));

        // Follow the path to align the robot
        follower.followPath(alignmentPath);

        // Switch to auto-align mode
        isAutoAligning = true;
    }
}