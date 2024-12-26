package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
import org.firstinspires.ftc.teamcode.HardwareClasses.Pivot;

@TeleOp(name = "Pivot Test")
public class PivotTest extends LinearOpMode {

    private Pivot pivot;
    private LinearSlide linearSlide;

    @Override
    public void runOpMode() {

        pivot = new Pivot(hardwareMap);
        String[] motorNames = {"slideMotor"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.REVERSE};
        LinearSlide linearSlide = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 38); // Example ticksPerInch and limits

        waitForStart();

        while (opModeIsActive()) {
            // Check for button presses and set target angles
            if (gamepad1.dpad_up) {
                pivot.movePivotToAngle(0);  // Move to 0 degrees
            } else if (gamepad1.dpad_down) {
                pivot.movePivotToAngle(70);  // Move to 90 degrees
            } else if (gamepad1.dpad_left) {
                pivot.movePivotToAngle(pivot.getPivotAngle() + 10);
            } else if (gamepad1.dpad_right) {
                pivot.movePivotToAngle(pivot.getPivotAngle() - 10);
            } else if (gamepad1.a) {
                linearSlide.moveSlidesToPositionInches(10);
            } else if (gamepad1.b) {
                linearSlide.moveSlidesToPositionInches(0);
            }

            pivot.update();
            linearSlide.update();

            telemetry.addData("Position Inches", linearSlide.slidesPositionInches());
            telemetry.addData("Is Busy", linearSlide.isSlideMotorsBusy());
            telemetry.addData("Current Angle", pivot.getPivotAngle());
            telemetry.update();
        }
    }
}