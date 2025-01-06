package org.firstinspires.ftc.teamcode.Auto;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClasses.DepositAssembly;
import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeAssemblyClaw;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Scrimmage 2 Auto Red")
public class Scrimmage2Auto2 extends OpMode {

    private IntakeAssemblyClaw intakeAssembly;
    private DepositAssembly depositAssembly;
    private LinearSlide linearSlides;
    // Initialize path following stuff
    private Follower follower;
    private Path toChamber, toSpike1Grab, toSpike1Give, toSpike2Grab, toSpike2Give, toSpike3Grab, toSpike3Give, toHumanPlayer, toPark;
    private Timer pathTimer;
    private int pathState;
    private int times;
    // Initialize telemetry and any other subsystems and variables
    private Telemetry telemetryA;
    private int cycles = 0;

    @Override
    public void init() {
        // Initialize path stuff with hardwareMap
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(-12, 63.5, Math.toRadians(-90)));
        follower.setMaxPower(0.8);
        pathTimer = new Timer();
        pathState = 0;

        String[] motorNames = {"slideMotorLeft", "slideMotorRight"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE};
        linearSlides = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 37.5); // Example ticksPerInch and limits

        intakeAssembly = new IntakeAssemblyClaw(hardwareMap);

        depositAssembly = new DepositAssembly(hardwareMap);

        intakeAssembly.OpenClaw();
        intakeAssembly.PivotClawUp();
        intakeAssembly.RotateClaw0();
        intakeAssembly.RetractSlidesFull();
        depositAssembly.CloseOuttakeClaw();
        depositAssembly.ScoreSpecimen();

        // Initialize telemetry
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryA.addData("Status:", "initialized");
        telemetryA.update();
    }

    @Override
    public void init_loop() {

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
        linearSlides.update();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                toChamber = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-6, 36, Point.CARTESIAN)));
                toChamber.setConstantHeadingInterpolation(Math.toRadians(-90));
                follower.followPath(toChamber, true);
                linearSlides.moveSlidesToPositionInches(19);
                setPathState(1);
                times = 0;
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(1);
                        times = 1;
                    }

                    linearSlides.moveSlidesToPositionInches(27);

                    if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                        depositAssembly.OpenOuttakeClaw();
                        linearSlides.moveSlidesToPositionInches(13);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                toSpike1Grab = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-26.5, 44.5, Point.CARTESIAN)));
                toSpike1Grab.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45));
                follower.followPath(toSpike1Grab, true);
                setPathState(3);
                times = 0;
                break;

            case 3:
                if (follower.isBusy()) {
                    if (follower.getCurrentTValue() > 0.8) {
                        intakeAssembly.ExtendSlidesFull();
                        intakeAssembly.PivotClawDown();
                        intakeAssembly.OpenClaw();
                        intakeAssembly.RotateClawToPos(0.15);
                        depositAssembly.GrabSpecimen();
                        linearSlides.moveSlidesToPositionInches(0);
                    }
                }
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(3);
                        times = 1;
                    }

                    intakeAssembly.CloseClaw();

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        intakeAssembly.PivotClawUp();
                        setPathState(4);
                    }
                }
                break;

            case 4:
                toSpike1Give = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-40, 45, Point.CARTESIAN)));
                toSpike1Give.setConstantHeadingInterpolation(Math.toRadians(45));
                follower.followPath(toSpike1Give, true);
                setPathState(5);
                times = 0;
                break;

            case 5:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(5);
                        times = 1;
                    }

                    intakeAssembly.OpenClaw();

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        intakeAssembly.PivotClawDown();
                        intakeAssembly.ExtendSlidesToPos(0.5);
                        intakeAssembly.RotateClaw90();
                        setPathState(6);
                    }
                }
                break;

            case 6:
                toSpike2Grab = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-39, 29, Point.CARTESIAN)));
                toSpike2Grab.setConstantHeadingInterpolation(Math.toRadians(180));
                follower.followPath(toSpike2Grab, true);
                setPathState(7);
                times = 0;
                break;

            case 7:
                if (follower.isBusy()) {
                    if (follower.getCurrentTValue() > 0.8) {
                        intakeAssembly.ExtendSlidesToPos(0.5);
                    }
                }
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(7);
                        times = 1;
                    }

                    intakeAssembly.CloseClaw();

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        intakeAssembly.PivotClawUp();
                        intakeAssembly.ExtendSlidesToPos(0.5);
                        setPathState(8);
                    }
                }
                break;

            case 8:
                toSpike2Give = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-40, 45, Point.CARTESIAN)));
                toSpike2Give.setConstantHeadingInterpolation(Math.toRadians(45));
                follower.followPath(toSpike2Give, true);
                setPathState(9);
                times = 0;
                break;

            case 9:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(9);
                        times = 1;
                    }

                    intakeAssembly.OpenClaw();

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        intakeAssembly.PivotClawDown();
                        intakeAssembly.ExtendSlidesToPos(0.4);
                        setPathState(10);
                    }
                }
                break;

            case 10:
                toSpike3Grab = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-50, 29, Point.CARTESIAN)));
                toSpike3Grab.setConstantHeadingInterpolation(Math.toRadians(180));
                follower.followPath(toSpike3Grab, true);
                setPathState(11);
                times = 0;
                break;

            case 11:
                if (follower.isBusy()) {
                    if (follower.getCurrentTValue() > 0.4) {
                        intakeAssembly.ExtendSlidesToPos(0.5);
                    }
                }
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(11);
                        times = 1;
                    }

                    intakeAssembly.CloseClaw();

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        intakeAssembly.PivotClawUp();
                        intakeAssembly.ExtendSlidesToPos(0.35);
                        setPathState(12);
                    }
                }
                break;

            case 12:
                toSpike3Give = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-40, 45, Point.CARTESIAN)));
                toSpike3Give.setConstantHeadingInterpolation(Math.toRadians(45));
                follower.followPath(toSpike3Give, true);
                setPathState(13);
                times = 0;
                break;

            case 13:
                if (follower.isBusy()) {
                    if (follower.getCurrentTValue() > 0.8) {
                        intakeAssembly.ExtendSlidesToPos(0.5);
                    }
                }
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(13);
                        times = 1;
                    }

                    intakeAssembly.OpenClaw();

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        intakeAssembly.ExtendSlidesToPos(0.35);
                        intakeAssembly.RotateClaw0();
                        intakeAssembly.PivotClawUp();
                        setPathState(14);
                    }
                }
                break;

            case 14:
                toHumanPlayer = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-38, 56 + cycles, Point.CARTESIAN)));
                toHumanPlayer.setConstantHeadingInterpolation(Math.toRadians(-90));
                follower.followPath(toHumanPlayer, true);
                setPathState(15);
                times = 0;
                break;

            case 15:
                if (follower.isBusy()) {
                    if (follower.getCurrentTValue() > 0.1) {
                        depositAssembly.GrabSpecimen();
                        linearSlides.moveSlidesToPositionInches(0);
                    }
                }
                if (!follower.isBusy()) {
                    if (cycles == 2) {
                        requestOpModeStop();
                    }
                    if (times == 0) {
                        setPathState(15);
                        times = 1;
                    }

                    depositAssembly.CloseOuttakeClaw();

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        depositAssembly.ScoreSpecimen();
                        linearSlides.moveSlidesToPositionInches(19);
                        setPathState(16);
                    }
                }
                break;

            case 16:
                toChamber = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-5 - (cycles * 0.5), 36.6, Point.CARTESIAN)));
                toChamber.setConstantHeadingInterpolation(Math.toRadians(-90));
                follower.followPath(toChamber, true);
                setPathState(17);
                times = 0;
                break;

            case 17:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(17);
                        times = 1;
                    }

                    linearSlides.moveSlidesToPositionInches(27);

                    if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                        depositAssembly.OpenOuttakeClaw();
                        linearSlides.moveSlidesToPositionInches(18);
                        cycles++;
                        if (cycles >= 4) {
                            setPathState(-1);
                        } else {
                            setPathState(14);
                        }
                    }
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