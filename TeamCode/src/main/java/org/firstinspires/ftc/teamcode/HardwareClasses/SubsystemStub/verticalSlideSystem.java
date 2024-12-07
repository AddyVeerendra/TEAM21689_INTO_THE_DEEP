package org.firstinspires.ftc.teamcode.HardwareClasses.SubsystemStub;

import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class verticalSlideSystem {
    LinearSlide LeftVerticalSlide;
    LinearSlide RightVerticalSlide;

    public verticalSlideSystem(HardwareMap hardwareMap) {
        LeftVerticalSlide = new LinearSlide(hardwareMap, new String[]{"leftSlideMotor"}, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD},
                76.51, 0, 44);

        RightVerticalSlide = new LinearSlide(hardwareMap, new String[]{"rightSlideMotor"}, new DcMotorSimple.Direction[]{DcMotorSimple.Direction.FORWARD},
                1000, 0, 44);
    }


    public void MaxSlideExtension() {
        LeftVerticalSlide.moveSlidesToPositionInches(LeftVerticalSlide.getMaxExtensionInches());
        RightVerticalSlide.moveSlidesToPositionInches(RightVerticalSlide.getMaxExtensionInches());
    }

    public void HighBasketExtension() {
        LeftVerticalSlide.moveSlidesToPositionInches(LeftVerticalSlide.getMaxExtensionInches());
        RightVerticalSlide.moveSlidesToPositionInches(RightVerticalSlide.getMaxExtensionInches());
    }

    public void LowBasketExtension() {
        LeftVerticalSlide.moveSlidesToPositionInches(LeftVerticalSlide.getMaxExtensionInches());
        RightVerticalSlide.moveSlidesToPositionInches(RightVerticalSlide.getMaxExtensionInches());
    }

    public void MinSlideExtension() {
        LeftVerticalSlide.moveSlidesToPositionInches(LeftVerticalSlide.getMinExtensionInches());
        RightVerticalSlide.moveSlidesToPositionInches(RightVerticalSlide.getMinExtensionInches());
    }


}