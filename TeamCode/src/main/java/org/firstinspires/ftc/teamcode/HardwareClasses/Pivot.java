package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Pivot {
    private static final double TICKS_PER_REV = 384.5;  // Motor ticks per revolution
    private static final double GEAR_REDUCTION = 28.0;  // Gear reduction
    private static final double TICKS_PER_OUTPUT_REV = TICKS_PER_REV * GEAR_REDUCTION;  // Total ticks per output revolution
    private final DcMotorEx motor;
    double lastSetPivotAngle = 0;
    boolean pivotMotorBusy = false;

    public Pivot(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "pivotMotor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void movePivotToAngle(double targetAngle) {
        lastSetPivotAngle = targetAngle;

        double error = targetAngle - getPivotAngle();

        if (Math.abs(error) > 1) { // Ensure error threshold is handled correctly
            motor.setPower(Range.clip((error * 0.04), -1, 1));
            pivotMotorBusy = true;
        } else {
            motor.setPower(0);
            pivotMotorBusy = false;
        }
    }

    public void correctPivotPosition() {
        if (!pivotMotorBusy) {
            double error = lastSetPivotAngle - getPivotAngle();
            motor.setPower(Range.clip((error * 0.15), -1, 1));
        }
    }

    public void update() {
        if (isPivotMotorBusy()) {
            movePivotToAngle(lastSetPivotAngle);
        } else {
            correctPivotPosition();
        }
    }

    public boolean isPivotMotorBusy() {
        return pivotMotorBusy;
    }

    public double getPivotAngle() {
        // Get the current position of the motor in ticks
        double currentTicks = motor.getCurrentPosition();

        // Calculate the angle in degrees based on the number of ticks per output revolution
        return (currentTicks / TICKS_PER_OUTPUT_REV) * 360.0;
    }
}