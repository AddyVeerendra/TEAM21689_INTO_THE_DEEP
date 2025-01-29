package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.pedropathing.localization.constants.PinpointConstants;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
//        PinpointConstants.forwardY = -4.75;
//        PinpointConstants.strafeX = -4.75;
//        PinpointConstants.distanceUnit = DistanceUnit.INCH;
//        PinpointConstants.hardwareMapName = "pinpoint";
//        PinpointConstants.useYawScalar = false;
//        PinpointConstants.yawScalar = 1.0;
//        PinpointConstants.useCustomEncoderResolution = false;
//        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;
//        PinpointConstants.customEncoderResolution = 13.26291192; //NOT USING
//        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
//        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        ThreeWheelIMUConstants.forwardTicksToInches = 0.0029;
        ThreeWheelIMUConstants.strafeTicksToInches = -0.0029;
        ThreeWheelIMUConstants.turnTicksToInches = 0.0029;
        ThreeWheelIMUConstants.leftY = 4.75;
        ThreeWheelIMUConstants.rightY = -4.75;
        ThreeWheelIMUConstants.strafeX = 5.75;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "rightRear";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "leftFront";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "leftRear";
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    }
}




