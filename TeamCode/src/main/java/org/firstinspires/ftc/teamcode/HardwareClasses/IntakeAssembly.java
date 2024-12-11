package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeAssembly {
    private Servo intakePivot;
    private Servo intakeRotate;
    private Servo intakeClaw;
    private Servo intakeSlidesLeft;

    public IntakeAssembly(HardwareMap hardwareMap) {
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        intakePivot.setDirection(Servo.Direction.REVERSE);
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeSlidesLeft = hardwareMap.get(Servo.class, "intakeSlidesLeft");
    }

    public void OpenClaw() {
        intakeClaw.setPosition(1);
    }

    public void CloseClaw() {
        intakeClaw.setPosition(0.75);
    }

    public void RotateClaw0() {
        intakeClaw.setPosition(0);
    }

    public void RotateClaw45() {
        intakeClaw.setPosition(0.15);
    }

    public void RotateClaw90() {
        intakeClaw.setPosition(0.3);
    }

    public void PivotClawDown() {
        intakePivot.setPosition(0.0);
    }

    public void PivotClawUp() {
        intakePivot.setPosition(0.65);
    }

    public void PivotClawMid() {
        intakePivot.setPosition(0.35);
    }

    public void ExtendSlidesFull() {
        intakeClaw.setPosition(0.7);
    }

    public void RetractSlidesFull() {
        intakeClaw.setPosition(0.3);
    }

    public void ExtendSlidesToPos(double pos) {
        intakeClaw.setPosition(pos);
    }
}