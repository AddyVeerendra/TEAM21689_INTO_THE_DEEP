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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareClasses.DepositAssembly;
import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeAssemblyClaw;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "Qual 1 Auto Specimen")
public class QualAutoSpecimen extends OpMode {

    public static final int TO_SPIKE_TWO = 5;
    public static final int BACK_TO_HUMAN_PLAYER_FROM_SPIKE_1 = 4;
    public static final int WAIT_FOLLOWER_TO_NOT_BE_BUSY = 6;
    private IntakeAssemblyClaw intakeAssembly;
    private DepositAssembly depositAssembly;
    private LinearSlide linearSlides;
    private DistanceSensor distanceSensorFront;
    private DistanceSensor distanceSensorBack;
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
        follower.setStartingPose(new Pose(12, -61.5, Math.toRadians(90)));
        follower.setMaxPower(0.7);
        pathTimer = new Timer();
        pathState = 0;

        String[] motorNames = {"slideMotorLeft", "slideMotorRight"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE};
        linearSlides = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 37.5); // Example ticksPerInch and limits

        intakeAssembly = new IntakeAssemblyClaw(hardwareMap);

        depositAssembly = new DepositAssembly(hardwareMap);

        distanceSensorFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        distanceSensorBack = hardwareMap.get(DistanceSensor.class, "distanceBack");

        intakeAssembly.OpenClaw();
        intakeAssembly.PivotClawUp();
        intakeAssembly.RotateClaw0();
        intakeAssembly.RetractSlidesFull();
        intakeAssembly.LockIntake();
        depositAssembly.CloseOuttakeClaw();
        depositAssembly.Hang();

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
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                toChamber = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(2.5, -40, Point.CARTESIAN)));
                toChamber.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toChamber, false);
                linearSlides.moveSlidesToPositionInches(16);
                depositAssembly.ScoreSpecimen();
                setPathState(1);
                times = 0;
                break;

            case 1:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(1);
                        times = 1;
                    }

                    double distance = distanceSensorFront.getDistance(DistanceUnit.INCH);

                    if (distance > 10.5 && distanceTimes == 0 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        // Move forward slowly
                        follower.startTeleopDrive();
                        follower.setTeleOpMovementVectors(0.3, 0, 0); // Adjust the speed as needed
                    } else if (distance < 10 && distanceTimes == 0 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        // Move backward slowly
                        follower.startTeleopDrive();
                        follower.setTeleOpMovementVectors(-0.3, 0, 0); // Adjust the speed as needed
                    } else {
                        if (distanceTimes == 0) {
                            follower.breakFollowing();
                            setPathState(1);
                            distanceTimes = 1;
                        }
                        linearSlides.setKP(0.003);
                        linearSlides.moveSlidesToPositionInches(7);

                        if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                            linearSlides.setKP(0.005);
                            depositAssembly.OpenOuttakeClaw();
                            linearSlides.moveSlidesToPositionInches(6);
                            setPathState(2);
                        }
                    }
                }
                break;

            case 2:
                distanceTimes = 0;
                follower.setMaxPower(1);
                toSpike1Grab = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(45, -60, Point.CARTESIAN),
                        new Point(36, -12, Point.CARTESIAN)));
                toSpike1Grab.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toSpike1Grab, true);
                setPathState(3);
                times = 0;
                break;

            case 3:
                if (follower.isBusy()) {
                    if (follower.getCurrentTValue() > 0.5) {
                        depositAssembly.GrabSpecimen();
                        linearSlides.moveSlidesToPositionInches(2);
                    }
                }
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;

            case 4:
                telemetryA.addData("In BACK_TO_HUMAN_PLAYER_FROM", "true");
                telemetryA.update();
                toSpike1Give = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(50, -12, Point.CARTESIAN),
                        new Point(46, -55, Point.CARTESIAN)));
                toSpike1Give.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toSpike1Give, true);
                setPathState(5);
                times = 0;
                break;

            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;

            case 6:
                telemetryA.addData("In TO_SPIKE_TWO", "true");
                telemetryA.update();
                toSpike2Grab = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(48, -12, Point.CARTESIAN)));
                toSpike2Grab.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toSpike2Grab, true);
                telemetryA.addData("Path State", pathState);
                telemetryA.update();
                setPathState(7);
                times = 0;
                break;

            case 7:
                telemetryA.addData("In WAIT_FOLLOWER_TO_NOT_BE_BUSY", "true");
                telemetryA.update();
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(7);
                        times = 1;
                    }

                    setPathState(8);
                }
                break;

            case 8:
                telemetryA.addData("In 7", "true");
                telemetryA.update();
                toSpike2Give = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(62, -12, Point.CARTESIAN),
                        new Point(58, -55, Point.CARTESIAN)));
                toSpike2Give.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toSpike2Give, true);
                setPathState(9);
                times = 0;
                break;

            case 9:
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;

            case 10:
                follower.setMaxPower(0.7);
                toHumanPlayer1 = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(36, -40, Point.CARTESIAN)));
                toHumanPlayer1.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toHumanPlayer1, true);
                setPathState(11);
                times = 0;
                break;

            case 11:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(11);
                        times = 1;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        setPathState(12);
                    }
                }
                break;

            case 12:
                toHumanPlayer2 = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(36, -53.5, Point.CARTESIAN)));
                toHumanPlayer2.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toHumanPlayer2, true);
                setPathState(13);
                times = 0;
                break;

            case 13:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(13);
                        times = 1;
                    }

                    double distance = distanceSensorBack.getDistance(DistanceUnit.INCH);

                    if (distance > 7 && distanceTimes == 0 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        // Move forward slowly
                        follower.startTeleopDrive();
                        follower.setTeleOpMovementVectors(-0.3, 0, 0); // Adjust the speed as needed
                    } else if (distance < 6.5 && distanceTimes == 0 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        // Move backward slowly
                        follower.startTeleopDrive();
                        follower.setTeleOpMovementVectors(0.3, 0, 0); // Adjust the speed as needed
                    } else {
                        if (distanceTimes == 0) {
                            follower.breakFollowing();
                            setPathState(13);
                            distanceTimes = 1;
                        }

                        depositAssembly.CloseOuttakeClaw();

                        if (pathTimer.getElapsedTimeSeconds() > 0.35) {
                            depositAssembly.ScoreSpecimen();
                            linearSlides.moveSlidesToPositionInches(16);
                            setPathState(14);
                        }
                    }
                }
                break;

            case 14:
                distanceTimes = 0;
                toChamber = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(30, -50, Point.CARTESIAN),
                        new Point(5 + (cycles * 1.5), -40, Point.CARTESIAN)));
                toChamber.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toChamber, false);
                setPathState(15);
                times = 0;
                break;

            case 15:
                if (follower.isBusy()) {
                    if (follower.getCurrentTValue() > 0.2) {
                        depositAssembly.ScoreSpecimen();
                    }
                }
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(15);
                        times = 1;
                    }

                    double distance = distanceSensorFront.getDistance(DistanceUnit.INCH);

                    if (distance > 10 && distanceTimes == 0 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        // Move forward slowly
                        follower.startTeleopDrive();
                        follower.setTeleOpMovementVectors(0.3, 0, 0); // Adjust the speed as needed
                    } else if (distance < 9.5 && distanceTimes == 0 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        // Move backward slowly
                        follower.startTeleopDrive();
                        follower.setTeleOpMovementVectors(-0.3, 0, 0); // Adjust the speed as needed
                    } else {
                        if (distanceTimes == 0) {
                            follower.breakFollowing();
                            setPathState(15);
                            distanceTimes = 1;
                        }
                        linearSlides.setKP(0.003);
                        linearSlides.moveSlidesToPositionInches(7);

                        if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                            linearSlides.setKP(0.005);
                            depositAssembly.OpenOuttakeClaw();
                            linearSlides.moveSlidesToPositionInches(2);
                            if (cycles < 2) {
                                setPathState(16);
                            } else {
                                depositAssembly.Hang();
                                linearSlides.moveSlidesToPositionInches(0);
                                setPathState(18);
                            }
                        }
                    }
                }
                break;

            case 16:
                distanceTimes = 0;
                toHumanPlayer1 = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(36, -40, Point.CARTESIAN)));
                toHumanPlayer1.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toHumanPlayer1, true);
                depositAssembly.GrabSpecimen();
                setPathState(17);
                times = 0;
                break;

            case 17:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(17);
                        times = 1;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        setPathState(18);
                    }
                }
                break;

            case 18:
                toHumanPlayer2 = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(36, -53.5, Point.CARTESIAN)));
                toHumanPlayer2.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(toHumanPlayer2, true);
                setPathState(19);
                times = 0;
                break;

            case 19:
                if (!follower.isBusy()) {
                    if (cycles == 2) {
                        requestOpModeStop();
                    }

                    if (times == 0) {
                        setPathState(19);
                        times = 1;
                    }

                    double distance = distanceSensorBack.getDistance(DistanceUnit.INCH);

                    if (distance > 7 && distanceTimes == 0 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        // Move forward slowly
                        follower.startTeleopDrive();
                        follower.setTeleOpMovementVectors(-0.3, 0, 0); // Adjust the speed as needed
                    } else if (distance < 6.5 && distanceTimes == 0 && pathTimer.getElapsedTimeSeconds() < 1.5) {
                        // Move backward slowly
                        follower.startTeleopDrive();
                        follower.setTeleOpMovementVectors(0.3, 0, 0); // Adjust the speed as needed
                    } else {
                        if (distanceTimes == 0) {
                            follower.breakFollowing();
                            setPathState(19);
                            distanceTimes = 1;
                        }

                        depositAssembly.CloseOuttakeClaw();

                        if (pathTimer.getElapsedTimeSeconds() > 0.35) {
                            linearSlides.moveSlidesToPositionInches(16);
                            cycles++;
                            distanceTimes = 0;
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
