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
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClasses.DepositAssembly;
import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeAssemblyClaw;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "A Beautiful 5 Spec Auto")
public class StatesAutoSpecimen extends OpMode {

    private IntakeAssemblyClaw intakeAssembly;
    private DepositAssembly depositAssembly;
    private LinearSlide linearSlides;
    private DistanceSensor distanceSensorFront;
    // Initialize path following stuff
    private Follower follower;
    private Path toChamber, toSpike1Grab, toSpike1Give, toSpike2Grab, toSpike2Give, toSpike3Grab, toSpike3Give, toHumanPlayer1, toHumanPlayer2, toPark;
    private Timer pathTimer;
    private int pathState;
    private int times;
    private int distanceTimes = 0;
    // Initialize telemetry and any other subsystems and variables
    private Telemetry telemetryA;
    private int cycles = 0;

    @Override
    public void init() {
        // Initialize path stuff with hardwareMap
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(12, -61, Math.toRadians(-90)));
        follower.setMaxPower(0.85);
        pathTimer = new Timer();
        pathState = 0;

        String[] motorNames = {"slideMotorLeft", "slideMotorRight"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE};
        linearSlides = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 37.5); // Example ticksPerInch and limits

        intakeAssembly = new IntakeAssemblyClaw(hardwareMap);

        depositAssembly = new DepositAssembly(hardwareMap);

        distanceSensorFront = hardwareMap.get(DistanceSensor.class, "distanceFront");

        intakeAssembly.OpenClaw();
        intakeAssembly.PivotClawUp();
        intakeAssembly.RotateClaw0();
        intakeAssembly.RetractSlidesFull();
        intakeAssembly.UnlockIntake();
        intakeAssembly.setOffset(0);
        depositAssembly.CloseOuttakeClaw();
        depositAssembly.HangAuto();

        // Initialize telemetry
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryA.addData("Status:", "initialized");
        telemetryA.update();
    }

    @Override
    public void init_loop() {
        intakeAssembly.update();
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
        intakeAssembly.update();

        if (follower.isRobotStuck()) {
            follower.breakFollowing();
        }
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                toChamber = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-5.5, -35, Point.CARTESIAN)));
                toChamber.setConstantHeadingInterpolation(Math.toRadians(-90));
                follower.followPath(toChamber, false);
                linearSlides.moveSlidesToPositionInches(13);
                depositAssembly.ScoreSpecimen();
                setPathState(1);
                times = 0;
                break;

            case 1:
                if (follower.getCurrentTValue() > 0.4) {
                    intakeAssembly.ExtendSlidesToPos(15);
                    intakeAssembly.IntakeFlickerUpMid();
                }
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(1);
                        follower.startTeleopDrive();
                        follower.setTeleOpMovementVectors(-0.4, 0, 0);
                        times = 1;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        follower.breakFollowing();
                        linearSlides.setKP(0.005);
                        linearSlides.moveSlidesToPositionInches(4);
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                        depositAssembly.OpenOuttakeClaw();
                        setPathState(2);
                    }
                }
                break;

            case 2:
                distanceTimes = 0;
                follower.setMaxPower(1);
                toSpike1Grab = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(25, -36, Point.CARTESIAN)));
                toSpike1Grab.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(30));
                follower.followPath(toSpike1Grab, true);
                setPathState(3);
                times = 0;
                break;

            case 3:
                if (follower.getCurrentTValue() > 0.5) {
                    depositAssembly.GrabSpecimen();
                    linearSlides.moveSlidesToPositionInches(0);
                    intakeAssembly.ExtendSlidesToPos(55);
                }
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(3);
                        times = 1;
                    }

                    intakeAssembly.IntakeFlickerDown();

                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        setPathState(4);
                    }
                }
                break;

            case 4:
                toSpike1Give = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(26, -38, Point.CARTESIAN)));
                toSpike1Give.setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(-28));
                follower.followPath(toSpike1Give, false);
                setPathState(5);
                times = 0;
                break;

            case 5:
                if (!follower.isBusy()) {
                    intakeAssembly.IntakeFlickerUpMid();
                    setPathState(6);
                }
                break;

            case 6:
                toSpike2Grab = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(36, -31, Point.CARTESIAN)));
                toSpike2Grab.setLinearHeadingInterpolation(Math.toRadians(-28), Math.toRadians(33));
                follower.followPath(toSpike2Grab, true);
                setPathState(7);
                times = 0;
                break;

            case 7:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(7);
                        times = 1;
                    }

                    intakeAssembly.IntakeFlickerDown();

                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        setPathState(8);
                    }
                }
                break;

            case 8:
                toSpike2Give = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(37, -41, Point.CARTESIAN)));
                toSpike2Give.setLinearHeadingInterpolation(Math.toRadians(33), Math.toRadians(-40));
                follower.followPath(toSpike2Give, false);
                setPathState(9);
                times = 0;
                break;

            case 9:
                if (!follower.isBusy()) {
                    intakeAssembly.IntakeFlickerUp();
                    setPathState(10);
                }
                break;

            case 10:
                toSpike3Grab = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(46, -36, Point.CARTESIAN)));
                toSpike3Grab.setLinearHeadingInterpolation(Math.toRadians(-40), Math.toRadians(35));
                follower.followPath(toSpike3Grab, true);
                setPathState(11);
                times = 0;
                break;

            case 11:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(11);
                        times = 1;
                    }

                    intakeAssembly.IntakeFlickerDown();

                    if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                        setPathState(12);
                    }
                }
                break;

            case 12:
                toSpike3Give = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(46, -41, Point.CARTESIAN)));
                toSpike3Give.setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(-40));
                follower.followPath(toSpike3Give, false);
                setPathState(13);
                times = 0;
                break;

            case 13:
                if (!follower.isBusy()) {
                    intakeAssembly.IntakeFlickerVertical();
                    setPathState(14);
                }
                break;

            case 14:
                follower.setMaxPower(0.85);
                toHumanPlayer1 = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(41, -51.5, Point.CARTESIAN)));
                toHumanPlayer1.setConstantHeadingInterpolation(Math.toRadians(-90));
                follower.followPath(toHumanPlayer1, false);
                intakeAssembly.ExtendSlidesToPos(15);
                setPathState(15);
                times = 0;
                break;

            case 15:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(15);
                        follower.startTeleopDrive();
                        follower.setTeleOpMovementVectors(0.4, 0, 0);
                        times = 1;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        follower.breakFollowing();
                        depositAssembly.CloseOuttakeClaw();
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                        linearSlides.moveSlidesToPositionInches(13);
                        setPathState(16);
                    }
                }
                break;

            case 16:
                distanceTimes = 0;
                toChamber = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(22, -50, Point.CARTESIAN),
                        new Point(-3 + (cycles * 2.5), -32.5, Point.CARTESIAN)));
                toChamber.setConstantHeadingInterpolation(Math.toRadians(-90));
                follower.followPath(toChamber, false);
                setPathState(17);
                times = 0;
                break;

            case 17:
                if (follower.isBusy()) {
                    if (follower.getCurrentTValue() > 0.3) {
                        depositAssembly.ScoreSpecimen();
                    }
                }
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(17);
                        follower.startTeleopDrive();
                        follower.setTeleOpMovementVectors(-0.4, 0, 0);
                        times = 1;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        follower.breakFollowing();
                        linearSlides.setKP(0.005);
                        linearSlides.moveSlidesToPositionInches(4);
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.6) {

                        depositAssembly.OpenOuttakeClaw();

                        if (cycles < 3) {
                            setPathState(18);
                            depositAssembly.GrabSpecimen();
                        } else {
                            depositAssembly.Hang();
                            setPathState(18);
                        }

                    }
                }
                break;

            case 18:
                if (cycles < 3) {
                    toHumanPlayer2 = new Path(new BezierCurve(
                            new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                            new Point(40, -35, Point.CARTESIAN),
                            new Point(40, -49.5, Point.CARTESIAN)));
                    toHumanPlayer2.setConstantHeadingInterpolation(Math.toRadians(-90));
                } else if (cycles == 3) {
                    toHumanPlayer2 = new Path(new BezierLine(
                            new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                            new Point(40, -49.5, Point.CARTESIAN)));
                    toHumanPlayer2.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-30), 0.75);
                    follower.setMaxPower(1);
                }
                follower.followPath(toHumanPlayer2, false);
                setPathState(19);
                times = 0;
                break;

            case 19:
                if (follower.getCurrentTValue() > 0.3) {
                    linearSlides.moveSlidesToPositionInches(0);
                }
                if (follower.getCurrentTValue() > 0.3 && cycles == 3) {
                    depositAssembly.GrabSampleFloor();
                    intakeAssembly.ExtendSlidesFull();
                    intakeAssembly.IntakeFlickerVertical();
                }
                if (!follower.isBusy()) {
                    if (cycles == 3) {
                        intakeAssembly.PivotClawUp();
                        depositAssembly.CloseOuttakeClaw();
                        setPathState(-1);
                        return;
                    }

                    if (!follower.isBusy()) {
                        if (times == 0) {
                            setPathState(19);
                            follower.startTeleopDrive();
                            follower.setTeleOpMovementVectors(0.4, 0, 0);
                            times = 1;
                        }

                        if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                            follower.breakFollowing();
                            depositAssembly.CloseOuttakeClaw();
                        }

                        if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                            linearSlides.moveSlidesToPositionInches(13);
                            cycles++;
                            setPathState(16);
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
