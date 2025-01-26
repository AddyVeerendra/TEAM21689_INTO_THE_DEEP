package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareClasses.DepositGripper;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
import org.firstinspires.ftc.teamcode.HardwareClasses.Pivot;

@TeleOp(name = "gripper test")
public class GripperTest extends LinearOpMode {
    private DepositGripper gripper;
    private LinearSlide linearSlide;
    private Pivot pivot;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private boolean isGripperOpen = false;
    private boolean isGripperRotated = false;
    private boolean toggleRightBumper = false;
    private boolean toggleLeftBumper = false;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        gripper = new DepositGripper(hardwareMap);
        pivot = new Pivot(hardwareMap);

        String[] motorNames = {"slideMotor"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.REVERSE};
        LinearSlide linearSlide = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 38); // Example ticksPerInch and limits

        gripper.CloseGripper();
        gripper.grabSpecimen();

        // Initialize mecanum drive motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set motor directions
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeInInit() && !isStopRequested()) {
            pivot.movePivotToAngle(90);
            pivot.update();
        }

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, check proximity and detect color
        while (opModeIsActive()) {
            // Mecanum drive control
            double y = -gamepad1.left_stick_y; // Invert Y axis
            double x = gamepad1.left_stick_x * 1.1; // Adjust for strafing power
            double rx = gamepad1.right_stick_x;

            // Calculate motor powers
            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            // Clip the motor powers to ensure they are within the range [-1, 1]
            frontLeftPower = Math.max(-1, Math.min(1, frontLeftPower));
            backLeftPower = Math.max(-1, Math.min(1, backLeftPower));
            frontRightPower = Math.max(-1, Math.min(1, frontRightPower));
            backRightPower = Math.max(-1, Math.min(1, backRightPower));

            // Set the motor powers
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

//            if (gamepad1.dpad_left) {
//                pivot.movePivotToAngle(15);
//                linearSlide.moveSlidesToPositionInches(19);
//                gripper.placeSample();
//            } else
//            } else if (gamepad1.dpad_down) {
//                pivot.movePivotToAngle(70);
//                linearSlide.moveSlidesToPositionInches(15);
//                gripper.grabSample();
//            else if (gamepad1.b) {
//                gripper.grabSampleFully();

            if (gamepad1.a) {
                pivot.movePivotToAngle(0);
                linearSlide.moveSlidesToPositionInches(2);
                gripper.grabSpecimen();
            } else if (gamepad1.b) {
                pivot.movePivotToAngle(45);
                linearSlide.moveSlidesToPositionInches(15);
                gripper.placeSpecimen();
            } else if (gamepad1.x) {
                pivot.movePivotToAngle(85);
                linearSlide.moveSlidesToPositionInches(8);
                gripper.grabSample();
            } else if (gamepad1.y) {
                pivot.movePivotToAngle(15);
                linearSlide.moveSlidesToPositionInches(19);
                gripper.placeSample();
            } else if (gamepad1.dpad_up) {
                pivot.movePivotToAngle(0);
                linearSlide.moveSlidesToPositionInches(5);
                gripper.grabSpecimen();
            } else if (gamepad1.dpad_left) {
                pivot.movePivotToAngle(0);
                linearSlide.moveSlidesToPositionInches(0);
                gripper.grabSpecimen();
            } else if (gamepad1.dpad_down) {
                pivot.movePivotToAngle(90);
                linearSlide.moveSlidesToPositionInches(0);
                gripper.grabSpecimen();
            }

            if (gamepad1.right_trigger > 0.5) {
                gripper.placeSpecimenFully();
                linearSlide.moveSlidesToPositionInches(8);
            } else if (gamepad1.left_trigger > 0.5) {
                pivot.movePivotToAngle(90);
                gripper.grabSampleFully();
            }

            // Toggle rotate gripper open/close using A button
            if (gamepad1.left_bumper && !toggleLeftBumper) {
                isGripperRotated = !isGripperRotated;
                toggleLeftBumper = true;
                if (isGripperRotated) {
                    gripper.rotateGripperToPos(0.35);
                } else {
                    gripper.rotateGripperToPos(0);
                }
            } else if (!gamepad1.left_bumper) {
                toggleLeftBumper = false;
            }

            // Toggle gripper open/close using A button
            if (gamepad1.right_bumper && !toggleRightBumper) {
                isGripperOpen = !isGripperOpen;
                toggleRightBumper = true;
                if (isGripperOpen) {
                    gripper.OpenGripper();
                } else {
                    gripper.CloseGripper();
                }
            } else if (!gamepad1.right_bumper) {
                toggleRightBumper = false;
            }

            linearSlide.update();
            pivot.update();
        }
    }
}