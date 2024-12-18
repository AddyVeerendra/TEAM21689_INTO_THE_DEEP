package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClasses.DepositAssembly;
import org.firstinspires.ftc.teamcode.HardwareClasses.DepositGripper;
import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeAssembly;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
import org.firstinspires.ftc.teamcode.HardwareClasses.Pivot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "Qual 1 Auto Specimen")
public class QualAutoSpecimen extends OpMode {

    private IntakeAssembly intakeAssembly;
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
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(12, -63.5, Math.toRadians(90)));
        follower.setMaxPower(0.7);
        pathTimer = new Timer();
        pathState = 0;

        String[] motorNames = {"slideMotorLeft", "slideMotorRight"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE};
        linearSlides = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 37.5); // Example ticksPerInch and limits

        intakeAssembly = new IntakeAssembly(hardwareMap);

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
                        new Point(6, -36, Point.CARTESIAN)));
                toChamber.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toChamber, true);
                linearSlides.moveSlidesToPositionInches(14);
                setPathState(1);
                times = 0;
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(1);
                        times = 1;
                    }

                    linearSlides.setKP(0.0025);
                    linearSlides.moveSlidesToPositionInches(9);

                    if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                        linearSlides.setKP(0.005);
                        depositAssembly.OpenOuttakeClaw();
                        linearSlides.moveSlidesToPositionInches(6);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                toSpike1Grab = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(30, -50, Point.CARTESIAN),
                        new Point(40, -12, Point.CARTESIAN)));
                toSpike1Grab.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toSpike1Grab, false);
                setPathState(3);
                times = 0;
                break;

            case 3:
                if (follower.isBusy()) {
                    if (follower.getCurrentTValue() > 0.5) {
                        depositAssembly.GrabSpecimen();
                        linearSlides.moveSlidesToPositionInches(0);
                    }
                }
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;

            case 4:
                toSpike1Give = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(46, -12, Point.CARTESIAN),
                        new Point(44, -55, Point.CARTESIAN)));
                toSpike1Give.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toSpike1Give, false);
                setPathState(5);
                times = 0;
                break;

            case 5:
                toSpike2Grab = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(40, -12, Point.CARTESIAN)));
                toSpike2Grab.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toSpike2Grab, false);
                setPathState(6);
                times = 0;
                break;

            case 6:
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;

            case 7:
                toSpike2Give = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(62, -12, Point.CARTESIAN),
                        new Point(48, -55, Point.CARTESIAN)));
                toSpike2Give.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toSpike2Give, false);
                setPathState(-1);
                times = 0;
                break;

            case 9:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(9);
                        times = 1;
                    }
                    intakeAssembly.PivotClawDown();
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
                        new Point(49, -28, Point.CARTESIAN)));
                toSpike3Grab.setConstantHeadingInterpolation(Math.toRadians(0));
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
                        intakeAssembly.PivotClawMid();
                        intakeAssembly.ExtendSlidesToPos(0.35);
                        setPathState(12);
                    }
                }
                break;

            case 12:
                toSpike3Give = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(40, -45, Point.CARTESIAN)));
                toSpike3Give.setConstantHeadingInterpolation(Math.toRadians(-45));
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

                    intakeAssembly.PivotClawDown();
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
                toHumanPlayer = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(30, -40, Point.CARTESIAN),
                        new Point(38, -57, Point.CARTESIAN)));
                toHumanPlayer.setConstantHeadingInterpolation(Math.toRadians(90));
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
                        new Point(5+(cycles * 0.5), -35, Point.CARTESIAN)));
                toChamber.setConstantHeadingInterpolation(Math.toRadians(90));
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