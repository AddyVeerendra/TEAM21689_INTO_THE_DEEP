package org.firstinspires.ftc.teamcode.HardwareClasses.SubsystemStub;

import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class verticalSlideSystem {
    LinearSlide LeftHorizontalSlide;
    LinearSlide RightHorizontalSlide;

    public verticalSlideSystem(HardwareMap hardwareMap) {
        LeftHorizontalSlide = new LinearSlide(hardwareMap, new String[]{"leftSlideMotor"}, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD},
                1000, 0, 40);

        RightHorizontalSlide = new LinearSlide(hardwareMap, new String[]{"rightSlideMotor"}, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD},
                1000, 0, 40);
    }

    public void MaxSlideExtension() {
        LeftHorizontalSlide.moveSlidesToPositionInches(LeftHorizontalSlide.getMaxExtensionInches());
        RightHorizontalSlide.moveSlidesToPositionInches(RightHorizontalSlide.getMaxExtensionInches());
    }

    public void MinSlideExtension() {
        LeftHorizontalSlide.moveSlidesToPositionInches(LeftHorizontalSlide.getMinExtensionInches());
        RightHorizontalSlide.moveSlidesToPositionInches(RightHorizontalSlide.getMinExtensionInches());
    }


}