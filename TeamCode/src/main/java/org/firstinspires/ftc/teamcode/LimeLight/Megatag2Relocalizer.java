package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Megatag2Relocalizer", group = "Sensor")
public class Megatag2Relocalizer extends LinearOpMode {

    IMU imu;
    private Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(IMU.class, "imu");

        limelight = hardwareMap.get(Limelight3A.class, "Limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(7);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        if (limelight.isConnected()) {
            telemetry.addData(">", "Limelight Connected");
        } else {
            telemetry.addData(">", "Limelight Not Connected");
        }
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Looking!");

            imu = hardwareMap.get(IMU.class, "imu");

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

            LLResult result = limelight.getLatestResult();

            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose_MT2();
                    telemetry.addData("Botpose", botpose.toString());
                    telemetry.update();
                }
            }
        }
        telemetry.update();
    }
}