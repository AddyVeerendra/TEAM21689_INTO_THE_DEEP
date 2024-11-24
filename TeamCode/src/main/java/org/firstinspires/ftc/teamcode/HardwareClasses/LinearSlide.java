package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.List;

public class LinearSlide {
    private final List<DcMotorEx> motors = new ArrayList<>();
    private final double maxExtensionInches;
    private final double minExtensionInches;
    private final double ticksPerInch; // Conversion factor from inches to motor ticks
    private double lastSetSlidePos = 0;
    private boolean slideMotorsBusy = false;

    // Constructor that accepts motor names, directions, and extension limits
    public LinearSlide(HardwareMap hardwareMap, String[] motorNames, DcMotorSimple.Direction[] directions,
                       double ticksPerInch, double minExtensionInches, double maxExtensionInches) {
        this.ticksPerInch = ticksPerInch;
        this.minExtensionInches = minExtensionInches;
        this.maxExtensionInches = maxExtensionInches;

        for (int i = 0; i < motorNames.length; i++) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, motorNames[i]);
            motor.setDirection(directions[i]);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motors.add(motor);
        }
    }

    // Convert inches to motor ticks
    private double inchesToTicks(double inches) {
        return inches * ticksPerInch;
    }

    // Move the slide to the target position in inches, with limits applied
    public void moveSlidesToPositionInches(double targetInches) {
        // Clamp target position within allowed range
        targetInches = Range.clip(targetInches, minExtensionInches, maxExtensionInches);

        // Convert the target position from inches to motor ticks
        double targetTicks = inchesToTicks(targetInches);

        // Move each motor to the calculated target position in ticks
        moveSlidesToPosition(targetTicks);
    }

    // Move the slides to the target position in ticks (internal method)
    private void moveSlidesToPosition(double targetTicks) {
        lastSetSlidePos = targetTicks;

        for (DcMotorEx motor : motors) {
            double error = targetTicks - motor.getCurrentPosition();

            if (Math.abs(error) > 20) { // Ensure error threshold is handled correctly
                double power = error * 0.005;
                motor.setPower(Range.clip(power, -1, 1)); // Adjust power direction
                slideMotorsBusy = true;
            } else {
                motor.setPower(0);
                slideMotorsBusy = false;
            }
        }
    }

    // Corrects the position of all motors
    public void correctSlidePositions() {
        if (!slideMotorsBusy) {
            for (DcMotorEx motor : motors) {
                double error = lastSetSlidePos - motor.getCurrentPosition();
                motor.setPower(Range.clip((error * 0.005), -0.5, 0.5));
            }
        }
    }

    // Updates the slide's position and correction
    public void update() {
        if (isSlideMotorsBusy()) {
            moveSlidesToPosition(lastSetSlidePos);
        } else {
            correctSlidePositions();
        }
    }

    // Checks if any motor is busy
    public boolean isSlideMotorsBusy() {
        return slideMotorsBusy;
    }

    // Get the average position of all motors (in inches)
    public double slidesPositionInches() {
        double totalTicks = 0;
        for (DcMotorEx motor : motors) {
            totalTicks += motor.getCurrentPosition();
        }
        return totalTicks / motors.size() / ticksPerInch; // Convert average ticks to inches
    }
}