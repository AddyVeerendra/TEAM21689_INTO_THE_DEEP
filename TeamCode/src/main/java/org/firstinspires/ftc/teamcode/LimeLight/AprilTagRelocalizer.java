package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.HardwareClasses.Limelight;

@TeleOp(name = "AprilTagRelocalizer", group = "Sensor")
public class AprilTagRelocalizer extends LinearOpMode {

    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections
            = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections
            = RevHubOrientationOnRobot.UsbFacingDirection.values();

    @Override
    public void runOpMode() throws InterruptedException {
        Limelight limelight = new Limelight(hardwareMap);

        IMU imu;
        int logoFacingDirectionPosition;
        int usbFacingDirectionPosition;
        boolean orientationIsValid = true;

        telemetry.setMsTransmissionInterval(11);
        limelight.getLimelight().setPollRateHz(11);
        limelight.pipelineSwitch(7);
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            imu = hardwareMap.get(IMU.class, "imu");
            logoFacingDirectionPosition = 0; // Up
            usbFacingDirectionPosition = 2; // Forward

            RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
            RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];

            try {
                RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
                imu.initialize(new IMU.Parameters(orientationOnRobot));
                orientationIsValid = true;
            } catch (IllegalArgumentException e) {
                orientationIsValid = false;
            }

            if (limelight.isConnected() && !limelight.getDetectorResults().isEmpty()) {
                double robotYaw = imu.getRobotYawPitchRollAngles().getYaw();
                limelight.updateRobotOrientation(robotYaw);
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    Pose3D Pose3D = limelight.getBotpose();
                    if (Pose3D != null) {
                        double x = Pose3D.getPosition().x;
                        double y = Pose3D.getPosition().y;
                        telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
                    }
                }
            }
        }
    }
}

