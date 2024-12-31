package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelIMUConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches = 0.0029;
        ThreeWheelIMUConstants.strafeTicksToInches = -0.0029;
        ThreeWheelIMUConstants.turnTicksToInches = 0.0029;
        ThreeWheelIMUConstants.leftY = 4.75;
        ThreeWheelIMUConstants.rightY = -4.75;
        ThreeWheelIMUConstants.strafeX = 4.75;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "rightBack";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "leftFront";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "leftBack";
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    }
}




