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

@Autonomous(name = "LeftSampleAutoV1")
public class LeftSampleAutoV1 extends OpMode {
    private IntakeAssemblyClaw intakeAssembly;
    private DepositAssembly depositAssembly;
    private LinearSlide linearSlides;
    // Initialize path following stuff
    private Follower follower;
    private Path toSpike1Grab, toBasket, toBasket2, toSpike2Grab, toSpike3Grab, toPark;
    private Timer pathTimer;
    private int pathState;
    private int times;
    // Initialize telemetry and any other subsystems and variables
    private Telemetry telemetryA;

    @Override
    public void init() {
        // Initialize path stuff with hardwareMap
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(-12, -61.5, Math.toRadians(-90)));
        follower.setMaxPower(0.6);
        pathTimer = new Timer();
        pathState = 0;

        String[] motorNames = {"slideMotorLeft", "slideMotorRight"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE};
        linearSlides = new LinearSlide(hardwareMap, motorNames, directions, 82.25625, 0, 37.5); // Example ticksPerInch and limits

        intakeAssembly = new IntakeAssemblyClaw(hardwareMap);

        depositAssembly = new DepositAssembly(hardwareMap);

        intakeAssembly.OpenClaw();
        intakeAssembly.PivotClawUp();
        intakeAssembly.RotateClaw0();
        intakeAssembly.RetractSlidesFull();
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
        intakeAssembly.update();
        autoPathUpdate();
        follower.telemetryDebug(telemetryA);
        linearSlides.update();
        telemetryA.addData("Path State", pathState);
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                toBasket = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-58, -57, Point.CARTESIAN)));
                toBasket.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-135));
                follower.followPath(toBasket, true);
                linearSlides.moveSlidesToPositionInches(30);
                depositAssembly.ScoreSampleFront();
                setPathState(1);
                times = 0;
                break;

            case 1:
                if (!follower.isBusy() && !linearSlides.isSlideMotorsBusy()) {
                    if (times == 0) {
                        setPathState(1);
                        times = 1;
                    }
                    depositAssembly.OpenOuttakeClaw();
                    intakeAssembly.ExtendSlidesToPos(10);

                    if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                        setPathState(2);
                    }
                }

                break;

            case 2:
                toSpike1Grab = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-48, -40, Point.CARTESIAN),
                        new Point(-48, -30, Point.CARTESIAN)));
                toSpike1Grab.setConstantHeadingInterpolation(Math.toRadians(-90));
                follower.followPath(toSpike1Grab, false);
                linearSlides.moveSlidesToPositionInches(1.75);
                depositAssembly.GrabSampleFloor();
                setPathState(3);
                times = 0;
                break;

            case 3:
                if (follower.getCurrentTValue() > 0.4) {
//                    intakeAssembly.ExtendSlidesFull();
//                    intakeAssembly.PivotClawDown();
//                    intakeAssembly.OpenClaw();
//                    intakeAssembly.RotateClaw0();
                }
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(3);
                        times = 1;
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        depositAssembly.CloseOuttakeClaw();
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        depositAssembly.ScoreSampleFront();
                        linearSlides.moveSlidesToPositionInches(30);
                        setPathState(6);
                    }

//                    if (pathTimer.getElapsedTimeSeconds() > 0.75) {
//                        intakeAssembly.PivotClawUp();
//                    }
//
//                    if (pathTimer.getElapsedTimeSeconds() > 1) {
//                        intakeAssembly.RotateClaw0();
//                        intakeAssembly.ExtendSlidesToPos(24);
//                        setPathState(4);
//                    }
                }
                break;

            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                    depositAssembly.CloseOuttakeClaw();
                    setPathState(5);
                }
                break;

            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                    intakeAssembly.OpenClaw();
                    linearSlides.moveSlidesToPositionInches(31);
                    depositAssembly.ScoreSample();
                    intakeAssembly.ExtendSlidesFull();
                    intakeAssembly.PivotClawDown();
                    intakeAssembly.OpenClaw();
                    intakeAssembly.RotateClaw0();
                    setPathState(6);
                }
                break;

            case 6:
                toBasket = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-52, -52, Point.CARTESIAN)));
                toBasket.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-135));
                follower.followPath(toBasket, true);
                setPathState(7);
                times = 0;
                break;

            case 7:
                if (!follower.isBusy() && !linearSlides.isSlideMotorsBusy()) {
                    toBasket2 = new Path(new BezierLine(
                            new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                            new Point(-58, -57, Point.CARTESIAN)));
                    toBasket2.setConstantHeadingInterpolation(Math.toRadians(-135));
                    follower.followPath(toBasket2, true);
                    setPathState(8);
                    times = 0;
                }
                break;


            case 8:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(8);
                        times = 1;
                    }
                    depositAssembly.OpenOuttakeClaw();

                    if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                        linearSlides.moveSlidesToPositionInches(1.5);
                        depositAssembly.GrabSampleFloor();
                        setPathState(9);
                    }
                }
                break;

            case 9:
                toSpike2Grab = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-59, -40, Point.CARTESIAN),
                        new Point(-59, -30, Point.CARTESIAN)));
                toSpike2Grab.setConstantHeadingInterpolation(Math.toRadians(-90));
                follower.followPath(toSpike2Grab, false);
                setPathState(10);
                times = 0;
                break;

            case 10:
                if (follower.getCurrentTValue() > 0.4) {
//                    linearSlides.moveSlidesToPositionInches(0);
//                    depositAssembly.TransferSample();
                }
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(10);
                        times = 1;
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        depositAssembly.CloseOuttakeClaw();
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        depositAssembly.ScoreSampleFront();
                        linearSlides.moveSlidesToPositionInches(30);
                        setPathState(13);
                    }

//                    if (pathTimer.getElapsedTimeSeconds() > 0.75) {
//                        intakeAssembly.PivotClawUp();
//                    }
//
//                    if (pathTimer.getElapsedTimeSeconds() > 1) {
//                        intakeAssembly.RotateClaw0();
//                        intakeAssembly.ExtendSlidesToPos(24);
//                        setPathState(11);
//                    }
                }
                break;

            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                    depositAssembly.CloseOuttakeClaw();
                    setPathState(12);
                }
                break;

            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                    intakeAssembly.OpenClaw();
                    linearSlides.moveSlidesToPositionInches(31);
                    depositAssembly.ScoreSample();
                    intakeAssembly.ExtendSlidesFull();
                    intakeAssembly.PivotClawDown();
                    intakeAssembly.OpenClaw();
                    intakeAssembly.RotateClaw135();
                    setPathState(13);
                }
                break;

            case 13:
                toBasket = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-52, -52, Point.CARTESIAN)));
                toBasket.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-135));
                follower.followPath(toBasket, true);
                setPathState(14);
                times = 0;
                break;

            case 14:
                if (!follower.isBusy() && !linearSlides.isSlideMotorsBusy()) {
                    toBasket2 = new Path(new BezierLine(
                            new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                            new Point(-58, -57, Point.CARTESIAN)));
                    toBasket2.setConstantHeadingInterpolation(Math.toRadians(-135));
                    follower.followPath(toBasket2, true);
                    setPathState(-1);
                    times = 0;
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(15);
                        times = 1;
                    }
                    depositAssembly.OpenOuttakeClaw();

                    if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                        setPathState(16);
                    }
                }
                break;

            case 16:
                toSpike3Grab = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-60, -51, Point.CARTESIAN)));
                toSpike3Grab.setConstantHeadingInterpolation(Math.toRadians(110));
                follower.followPath(toSpike3Grab, true);
                setPathState(17);
                times = 0;
                break;

            case 17:
                if (follower.getCurrentTValue() > 0.4) {
                    linearSlides.moveSlidesToPositionInches(0);
                    depositAssembly.TransferSample();
                }
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(17);
                        times = 1;
                    }
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        intakeAssembly.CloseClaw();
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 0.75) {
                        intakeAssembly.RotateClaw0();
                        intakeAssembly.PivotClawUp();
                    }

                    if (pathTimer.getElapsedTimeSeconds() > 1) {
                        intakeAssembly.ExtendSlidesToPos(24);
                        setPathState(18);
                    }
                }
                break;

            case 18:
                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                    depositAssembly.CloseOuttakeClaw();
                    setPathState(19);
                }
                break;

            case 19:
                if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                    intakeAssembly.OpenClaw();
                    linearSlides.moveSlidesToPositionInches(31);
                    depositAssembly.ScoreSample();
                    intakeAssembly.ExtendSlidesFull();
                    setPathState(20);
                }
                break;

            case 20:
                toBasket = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-52, -52, Point.CARTESIAN)));
                toBasket.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45));
                follower.followPath(toBasket, true);
                setPathState(21);
                times = 0;
                break;

            case 21:
                if (!follower.isBusy() && !linearSlides.isSlideMotorsBusy()) {
                    toBasket2 = new Path(new BezierLine(
                            new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                            new Point(-58, -57, Point.CARTESIAN)));
                    toBasket2.setConstantHeadingInterpolation(Math.toRadians(45));
                    follower.followPath(toBasket2, true);
                    setPathState(22);
                    times = 0;
                }
                break;

            case 22:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        setPathState(22);
                        times = 1;
                    }
                    depositAssembly.OpenOuttakeClaw();

                    if (pathTimer.getElapsedTimeSeconds() > 0.15) {
                        setPathState(23);
                    }
                }
                break;

            case 23:
                follower.setMaxPower(0.8);
                toPark = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(-50, -10, Point.CARTESIAN),
                        new Point(-25, -10, Point.CARTESIAN)));
                toPark.setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0));
                follower.followPath(toPark, true);
                setPathState(24);
                times = 0;
                break;

            case 24:
                if (follower.getCurrentTValue() > 0.2) {
                    linearSlides.moveSlidesToPositionInches(0);
                    depositAssembly.Hang();
                }
                if (!follower.isBusy() && !linearSlides.isSlideMotorsBusy()) {
                    intakeAssembly.RetractSlidesFull();
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
