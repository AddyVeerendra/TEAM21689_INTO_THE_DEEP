package org.firstinspires.ftc.teamcode.MiscTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "ITPPathTest", group = "Pedro Auto")
public class ITPPathTest extends OpMode {

    // Initialize path following stuff
    private Follower follower;
    private Path toChamber, toObservationZone, toBasketFromChamber, toBasket, toSpike1, toSpike2, toSpike3, toBasketIntakeArea;
    private Timer pathTimer;
    private int pathState;
    private int chamberCycleCount = 0;

    // Initialize telemetry and any other subsystems and variables
    private Telemetry telemetryA;

    @Override
    public void init() {
        // Initialize path stuff with hardwareMap
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(12, -63.5, Math.toRadians(90)));
        pathTimer = new Timer();
        pathState = 0;

        // Initialize telemetry
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryA.addData("Status:", "initialized");
        telemetryA.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autoPathUpdate();
        follower.telemetryDebug(telemetryA);
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                toChamber = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(0, -33, Point.CARTESIAN)));
                toChamber.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toChamber, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                if (chamberCycleCount < 5) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        toObservationZone = new Path(new BezierLine(
                                new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                                new Point(40, -60, Point.CARTESIAN)));
                        toObservationZone.setConstantHeadingInterpolation(Math.toRadians(90));
                        follower.followPath(toObservationZone, true);
                        setPathState(3);
                        chamberCycleCount++;
                    }
                } else {
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        toBasketFromChamber = new Path(new BezierCurve(
                                new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                                new Point(-60, -35, Point.CARTESIAN),
                                new Point(-65, -55, Point.CARTESIAN)));
                        toBasketFromChamber.setReversed(true);
                        follower.followPath(toBasketFromChamber, true);
                        setPathState(5);
                    }
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;

            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(0);
                }
                break;

            case 5:
                if (follower.getCurrentTValue() > 0.75) {
                    toBasketFromChamber.setConstantHeadingInterpolation(Math.toRadians(45));
                }
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            default:
                // No further action
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }
}