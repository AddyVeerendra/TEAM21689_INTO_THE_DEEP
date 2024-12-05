package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClasses.DepositGripper;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
import org.firstinspires.ftc.teamcode.HardwareClasses.Pivot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "AAAAA Auto BLUE")
public class ScrimmageAuto extends OpMode {

    public DepositGripper gripper;
    public LinearSlide linearSlide;
    public Pivot pivot;
    // Initialize path following stuff
    private Follower follower;
    private Path toChamber, toHumanPlayer, toBasket, toSpike1, toSpike2, toSpike3, toPark;
    private Timer pathTimer;
    private int pathState;
    private int times;
    // Initialize telemetry and any other subsystems and variables
    private Telemetry telemetryA;

    @Override
    public void init() {
        // Initialize path stuff with hardwareMap
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(12, -63.5, Math.toRadians(90)));
        follower.setMaxPower(0.5);
        pathTimer = new Timer();
        pathState = 0;

        gripper = new DepositGripper(hardwareMap);
        pivot = new Pivot(hardwareMap);

        String[] motorNames = {"slideMotor"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.REVERSE};
        linearSlide = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 38); // Example ticksPerInch and limits

        gripper.CloseGripper();
        gripper.grabSpecimen();

        // Initialize telemetry
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryA.addData("Status:", "initialized");
        telemetryA.update();
    }

    @Override
    public void init_loop() {
        pivot.movePivotToAngle(90);
        pivot.update();
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
        pivot.update();
        linearSlide.update();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                toChamber = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(0, -47, Point.CARTESIAN)));
                toChamber.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toChamber, true);
                pivot.movePivotToAngle(40);
                gripper.placeSpecimen();
                setPathState(1);
                times = 0;
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(1);
                        times = 1;
                    }
                    linearSlide.moveSlidesToPositionInches(15);

                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {

                        gripper.placeSpecimenFully();
                        linearSlide.moveSlidesToPositionInches(7);

                        if (pathTimer.getElapsedTimeSeconds() > 2.25) {
                            gripper.OpenGripper();
                            setPathState(2);
                        }
                    }
                }
                break;

            case 2:
                toHumanPlayer = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(48, -55, Point.CARTESIAN),
                        new Point(48, -62, Point.CARTESIAN)));
                toHumanPlayer.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toHumanPlayer, true);
                pivot.movePivotToAngle(0);
                linearSlide.moveSlidesToPositionInches(2);
                gripper.grabSpecimen();
                setPathState(3);
                times = 0;
                break;

            case 3:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(3);
                        times = 1;
                    }
                    gripper.CloseGripper();

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        setPathState(4);
                    }
                }
                break;

            case 4:
                toChamber = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(5, -47, Point.CARTESIAN)));
                toChamber.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toChamber, true);
                pivot.movePivotToAngle(40);
                gripper.placeSpecimen();
                setPathState(5);
                times = 0;
                break;

            case 5:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(5);
                        times = 1;
                    }
                    linearSlide.moveSlidesToPositionInches(15);

                    if (pathTimer.getElapsedTimeSeconds() > 1.5) {

                        gripper.placeSpecimenFully();
                        linearSlide.moveSlidesToPositionInches(7);

                        if (pathTimer.getElapsedTimeSeconds() > 2.25) {
                            gripper.OpenGripper();
                            setPathState(6);
                        }
                    }
                }
                break;

            case 6:
                pivot.movePivotToAngle(0);
                linearSlide.moveSlidesToPositionInches(2);
                gripper.grabSpecimen();
                setPathState(7);
                break;

            case 7:
                toSpike1 = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-58, -49, Point.CARTESIAN)));
                toSpike1.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toSpike1, true);
                setPathState(8);
                times = 0;
                break;

            case 8:
                if (follower.isBusy()) {
                    if (follower.getCurrentTValue() > 0.5) {
                        pivot.movePivotToAngle(90);
                        linearSlide.moveSlidesToPositionInches(8);
                        gripper.grabSample();
                    }
                }
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(8);
                        times = 1;
                    }
                    gripper.grabSampleFully();

                    if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                        gripper.CloseGripper();
                        setPathState(9);
                    }
                }
                break;

            case 9:
                toBasket = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-67, -60, Point.CARTESIAN)));
                toBasket.setConstantHeadingInterpolation(Math.toRadians(45));
                follower.followPath(toBasket, true);
                pivot.movePivotToAngle(15);
                linearSlide.moveSlidesToPositionInches(19);
                gripper.placeSample();
                setPathState(10);
                times = 0;
                break;

            case 10:
                if (!follower.isBusy() && !pivot.isPivotMotorBusy()) {
                    if (times == 0) {
                        setPathState(10);
                        times = 1;
                    }
                    gripper.OpenGripper();

                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        setPathState(11);
                    }
                }
                break;

            case 11:
                toSpike2 = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-70, -49, Point.CARTESIAN)));
                toSpike2.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toSpike2, true);
                pivot.movePivotToAngle(90);
                linearSlide.moveSlidesToPositionInches(8);
                gripper.grabSample();
                setPathState(12);
                times = 0;
                break;

            case 12:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(12);
                        times = 1;
                    }
                    gripper.grabSampleFully();

                    if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                        gripper.CloseGripper();
                        setPathState(13);
                    }
                }
                break;

            case 13:
                toBasket = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-67, -60, Point.CARTESIAN)));
                toBasket.setConstantHeadingInterpolation(Math.toRadians(45));
                follower.followPath(toBasket, true);
                pivot.movePivotToAngle(15);
                linearSlide.moveSlidesToPositionInches(19);
                gripper.placeSample();
                setPathState(14);
                times = 0;
                break;

            case 14:
                if (!follower.isBusy() && !pivot.isPivotMotorBusy()) {
                    if (times == 0) {
                        setPathState(14);
                        times = 1;
                    }
                    gripper.OpenGripper();

                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        setPathState(15);
                    }
                }
                break;

            case 15:
                toPark = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-45, 0, Point.CARTESIAN),
                        new Point(-20, 10, Point.CARTESIAN)));
                toPark.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180));
                follower.followPath(toPark, true);
                pivot.movePivotToAngle(0);
                setPathState(16);
                times = 0;
                break;

            case 16:
                if (follower.isBusy()) {
                    if (follower.getCurrentTValue() > 0.25) {
                        linearSlide.moveSlidesToPositionInches(0);
                        gripper.grabSpecimen();
                    }
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