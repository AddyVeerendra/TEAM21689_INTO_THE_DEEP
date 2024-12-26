package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class KickoffArm {
    private final DcMotorEx motor;

    private final Servo leftServo;
    private final Servo rightServo;

    double lastSetArmPos = 0;
    boolean armMotorBusy = false;

    public KickoffArm(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "arm");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftServo = hardwareMap.get(Servo.class, "leftGripper");
        rightServo = hardwareMap.get(Servo.class, "rightGripper");
        rightServo.setDirection(Servo.Direction.REVERSE);
    }

    public void moveArmToPosition(double targetPos) {
        lastSetArmPos = targetPos;

        double error = targetPos - motor.getCurrentPosition();

        if (Math.abs(error) > 20) { // Ensure error threshold is handled correctly
            double power = error * 0.0005;

            // Ensure motor power is not between -0.25 and 0.25
            if (power > -0.25 && power < 0.25) {
                power = power > 0 ? 0.25 : -0.25;
            }

            motor.setPower(Range.clip(power, -0.6, 0.6)); // Adjust power direction
            armMotorBusy = true;
        } else {
            motor.setPower(0);
            armMotorBusy = false;
        }
    }

    public void correctArmPosition() {
        if (!armMotorBusy) {
            double error = lastSetArmPos - motor.getCurrentPosition(); // Change error calculation
            motor.setPower(Range.clip((error * 0.001), -0.5, 0.5)); // Adjust power direction
        }
    }

    public void update() {
        if (isArmMotorBusy()) {
            moveArmToPosition(lastSetArmPos);
        } else {
            correctArmPosition();
        }
    }

    public boolean isArmMotorBusy() {
        return armMotorBusy;
    }

    public double armMotorPosition() {
        return motor.getCurrentPosition();
    }

    public void CloseGripperCompletely() {
        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }

    public void OpenGripperCompletely() {
        leftServo.setPosition(0.41);
        rightServo.setPosition(0.35);
    }

    public void OpenGripperAtBottom() {
        leftServo.setPosition(0.28);
        rightServo.setPosition(0.07);
    }

    public void OpenGripper(double amount) {
        leftServo.setPosition(leftServo.getPosition() + amount);
        rightServo.setPosition(rightServo.getPosition() + amount);
    }

    public void CloseGripper(double amount) {
        leftServo.setPosition(leftServo.getPosition() - amount);
        rightServo.setPosition(rightServo.getPosition() - amount);
    }
}