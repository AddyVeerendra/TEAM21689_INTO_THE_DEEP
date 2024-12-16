package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "zero Slides")
public class zeroSlides extends LinearOpMode {
    private LinearSlide linearSlides;
    String[] motorNames = {"slideMotorLeft", "slideMotorRight"};

    DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE};

    @Override
    public void runOpMode() throws InterruptedException {
        linearSlides = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 37.5); // Example ticksPerInch and limits
        waitForStart();
        linearSlides.zeroSlides(-20); //Change the value to the desired zero position
    }

}
