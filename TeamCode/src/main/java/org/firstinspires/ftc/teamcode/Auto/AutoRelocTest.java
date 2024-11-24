package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.PedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.PedroPathing.util.Timer;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@TeleOp
public class AutoRelocTest extends OpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double FEET_PER_METER = 3.28084;
    double tagsize = 0.0508;
    Pose tagLocation = new Pose(0, 70, Math.toRadians(180));

    // Initialize path following stuff
    private Follower follower;
    private Path forwards, backwards;
    private Timer pathTimer;
    private int pathState;

    @Override
    public void init() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        aprilTagDetectionPipeline.setDecimation(2);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        // Initialize path stuff with hardwareMap
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));
        pathTimer = new Timer();
        buildPaths();
        pathState = 0;
    }

    @Override
    public void start() {
        follower.followPath(forwards, false);
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autoPathUpdate();
        relocFromTag();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    setPathState(1);
                }
                break;

            case 1:
                follower.followPath(backwards, false);
                setPathState(2);
                break;

            case 2:
                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;

            case 3:
                follower.followPath(forwards, false);
                setPathState(0);
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
        forwards = new Path(new BezierLine(
                new Point(0, 0, Point.CARTESIAN),
                new Point(60, 0, Point.CARTESIAN)));

        backwards = new Path(new BezierLine(
                new Point(60, 0, Point.CARTESIAN),
                new Point(0, 0, Point.CARTESIAN)));
        backwards.setReversed(true);
    }

    public void relocFromTag() {
        if (gamepad1.square) {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if (detections != null) {
                // If we don't see any tags
                if (detections.isEmpty()) {
                    telemetry.addLine("We don't see any tags right now");
                } else { // We do see tags!
                    for (AprilTagDetection detection : detections) {
                        if (detection.id == 1) {
                            Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

                            // Calculate the robot's new estimated position
                            Pose tagPoseRelativeToRobot = new Pose(
                                    detection.pose.z * FEET_PER_METER,
                                    detection.pose.x * FEET_PER_METER,
                                    Math.toRadians(rot.firstAngle)
                            );

                            Pose diff = tagPoseRelativeToRobot;//tagLocation.subtract(tagPoseRelativeToRobot);
                            // ACTUALLY FIGURE OUT IF USING

                            follower.setCurrentPoseWithOffset(diff);
                            telemetry.addLine("Reloc successful");
                        }
                    }
                }
                telemetry.update();
            }
        }
    }
}