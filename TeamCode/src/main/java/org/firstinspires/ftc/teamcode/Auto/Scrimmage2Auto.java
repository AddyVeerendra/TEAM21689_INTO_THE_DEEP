//package org.firstinspires.ftc.teamcode.Auto;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.HardwareClasses.DepositAssembly;
//import org.firstinspires.ftc.teamcode.HardwareClasses.DepositGripper;
//import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeAssembly;
//import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
//import org.firstinspires.ftc.teamcode.HardwareClasses.Pivot;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
//
//@Autonomous(name = "Scrimmage 2 Auto Blue")
//public class Scrimmage2Auto extends OpMode {
//
//    private IntakeAssembly intakeAssembly;
//    private DepositAssembly depositAssembly;
//    private LinearSlide linearSlides;
//    // Initialize path following stuff
//    private Follower follower;
//    private Path toChamber, toSpike1Grab, toSpike1Give, toSpike2Grab, toSpike2Give, toSpike3Grab, toSpike3Give, toHumanPlayer, toPark;
//    private Timer pathTimer;
//    private int pathState;
//    private int times;
//    // Initialize telemetry and any other subsystems and variables
//    private Telemetry telemetryA;
//
//    @Override
//    public void init() {
//        // Initialize path stuff with hardwareMap
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(new Pose(12, -63.5, Math.toRadians(90)));
//        follower.setMaxPower(0.75);
//        pathTimer = new Timer();
//        pathState = 0;
//
//        String[] motorNames = {"slideMotorLeft", "slideMotorRight"};
//        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE};
//        linearSlides = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 37.5); // Example ticksPerInch and limits
//
//        intakeAssembly = new IntakeAssembly(hardwareMap);
//
//        depositAssembly = new DepositAssembly(hardwareMap);
//
//        intakeAssembly.OpenClaw();
//        intakeAssembly.PivotClawUp();
//        intakeAssembly.RotateClaw0();
//        intakeAssembly.RetractSlidesFull();
//        depositAssembly.CloseOuttakeClaw();
//        depositAssembly.InitSampleAuto();
//
//        // Initialize telemetry
//        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        telemetryA.addData("Status:", "initialized");
//        telemetryA.update();
//    }
//
//    @Override
//    public void init_loop() {
//
//    }
//
//    @Override
//    public void start() {
//        setPathState(0);
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        autoPathUpdate();
//        follower.telemetryDebug(telemetryA);
//        linearSlides.update();
//    }
//
//    public void autoPathUpdate() {
//        switch (pathState) {
//            case 0:
//                toChamber = new Path(new BezierLine(
//                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
//                        new Point(-12, -40, Point.CARTESIAN)));
//                toChamber.setConstantHeadingInterpolation(Math.toRadians(90));
//                follower.followPath(toChamber, true);
//                linearSlides.moveSlidesToPositionInches(18);
//                depositAssembly.ScoreSpecimen();
//                setPathState(1);
//                times = 0;
//                break;
//
//            case 1:
//                if (!follower.isBusy()) {
//                    if (times == 0) {
//                        setPathState(1);
//                        times = 1;
//                    }
//
//                    linearSlides.moveSlidesToPositionInches(19);
//
//                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//                        depositAssembly.OpenOuttakeClaw();
//                    }
//
//                    if (pathTimer.getElapsedTimeSeconds() > 0.75) {
//                        setPathState(2);
//                    }
//                }
//                break;
//
//            case 2:
//                toSpike1 = new Path(new BezierLine(
//                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
//                        new Point(-48, -48, Point.CARTESIAN)));
//                toSpike1.setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(90));
//                follower.followPath(toSpike1, true);
//                intakeAssembly.ExtendSlidesToPos(0.6);
//                intakeAssembly.PivotClawDown();
//                intakeAssembly.OpenClaw();
//                setPathState(3);
//                times = 0;
//                break;
//
//            case 3:
//                if (!follower.isBusy()) {
//                    if (times == 0) {
//                        setPathState(3);
//                        times = 1;
//                    }
//
//                    intakeAssembly.CloseClaw();
//
//                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
//                        intakeAssembly.PivotClawUp();
//                        intakeAssembly.RotateClaw0();
//                        setPathState(4);
//                    }
//                }
//                break;
//
//            case 4:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
//                    intakeAssembly.ExtendSlidesToPos(0.39);
//                    setPathState(5);
//                }
//                break;
//
//            case 5:
//                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
//                    depositAssembly.CloseOuttakeClaw();
//                    setPathState(6);
//                }
//                break;
//
//            case 6:
//                if (pathTimer.getElapsedTimeSeconds() > 0.15) {
//                    intakeAssembly.OpenClaw();
//                    setPathState(7);
//                }
//                break;
//
//            case 7:
//                if (pathTimer.getElapsedTimeSeconds() > 0.15) {
//                    toBasket = new Path(new BezierLine(
//                            new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
//                            new Point(-72, -60, Point.CARTESIAN)));
//                    toBasket.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45));
//                    follower.followPath(toBasket, true);
//                    linearSlides.moveSlidesToPositionInches(35);
//                    depositAssembly.ScoreSample();
//                    setPathState(8);
//                    times = 0;
//                }
//                break;
//
//            case 8:
//                linearSlides.moveSlidesToPositionInches(35);
//                depositAssembly.ScoreSample();
//                setPathState(9);
//                times = 0;
//                break;
//
//            case 9:
//                if (pathTimer.getElapsedTimeSeconds() > 2) {
//                    toBasket = new Path(new BezierLine(
//                            new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
//                            new Point(-72, -60, Point.CARTESIAN)));
//                    toBasket.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45));
//                    follower.followPath(toBasket, true);
//                    setPathState(10);
//                    times = 0;
//                }
//                break;
//
//            case 10:
//                if (!follower.isBusy()) {
//                    if (times == 0) {
//                        setPathState(1);
//                        times = 1;
//                    }
//                    depositAssembly.OpenOuttakeClaw();
//
//                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
//                        setPathState(-1);
//                    }
//                }
//                break;
//
//            default:
//                // No further action
//                break;
//        }
//    }
//
//    public void setPathState(int state) {
//        pathState = state;
//        pathTimer.resetTimer();
//    }
//}