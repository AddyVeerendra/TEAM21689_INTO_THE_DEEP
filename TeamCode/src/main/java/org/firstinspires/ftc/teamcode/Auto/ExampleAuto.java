package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClasses.RobotConstants;
import org.firstinspires.ftc.teamcode.PedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.PedroPathing.util.Timer;

@Autonomous(name = "Auto Base", group = "Pedro Auto")
public class ExampleAuto extends OpMode {
    // Import private final points from the Constants file
    private final Point examplePoint = RobotConstants.EXAMPLE_POINT;

    // Initialize path following stuff
    private Follower follower;
    private Path examplePath, examplePath2;
    private Timer pathTimer;
    private int pathState;

    // Initialize telemetry and any other subsystems and variables
    private Telemetry telemetryA;

    @Override
    public void init() {
        // Initialize path stuff with hardwareMap
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(15, -63, Math.toRadians(90)));
        pathTimer = new Timer();
        buildPaths();
        pathState = 0;

        // Initialize telemetry
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryA.addData("Status:", "initialized");
        telemetryA.update();
    }

    @Override
    public void start() {
        follower.followPath(examplePath, false);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autoPathUpdate();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                // Following the first path to the spike mark
                if (follower.getCurrentTValue() > 0.5) {
                    telemetryA.addLine("Halfway done with examplePath!");
                    telemetryA.update();
                }
                if (!follower.isBusy()) {
                    setPathState(1);
                }
                break;

            case 1:
                // Follow next path
                telemetryA.addLine("Done with examplePath!");
                telemetryA.update();
                follower.followPath(examplePath2, false);
                setPathState(2);
                break;

            case 2:
                // Following the first path to the spike mark
                if (follower.getCurrentTValue() > 0.5) {
                    telemetryA.addLine("Halfway done with examplePath!");
                    telemetryA.update();
                }
                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;
                
            case 3:
                // Follow next path
                telemetryA.addLine("Done with examplePath!");
                telemetryA.update();
                setPathState(-1);
                break;

            default:
                requestOpModeStop();
                // No further action
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        examplePath = new Path(new BezierLine(
                new Point(15, -63, Point.CARTESIAN),
                examplePoint));
        examplePath.setConstantHeadingInterpolation(Math.toRadians(90));

        examplePath2 = new Path(new BezierLine(
                examplePath.getLastControlPoint(),
                examplePath.getFirstControlPoint()));
        examplePath2.setReversed(true);
        examplePath2.setConstantHeadingInterpolation(Math.toRadians(90));
    }
}