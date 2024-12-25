package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeAssembly {
    private Servo intakePivot;
    private Servo intakeRotate;
    private Servo intakeClaw;
    private Servo intakeSlidesLeft;
    private Servo intakeLock;


    public IntakeAssembly(HardwareMap hardwareMap) {
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        intakePivot.setDirection(Servo.Direction.REVERSE);
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeSlidesLeft = hardwareMap.get(Servo.class, "intakeSlidesLeft");
        intakeLock = hardwareMap.get(Servo.class, "intakeLock");
    }

    public void LockIntake() {
        intakeLock.setPosition(0.26);
    }

    public void UnlockIntake() {
        intakeLock.setPosition(0.0);
    }

    public void OpenClaw() {
        intakeClaw.setPosition(0.75);
    }

    public void CloseClaw() {
        intakeClaw.setPosition(0.95);
    }

    public void RotateClaw0() {
        intakeRotate.setPosition(0);
    }

    public void RotateClaw45() {
        intakeRotate.setPosition(0.15);
    }

    public void RotateClaw90() {
        intakeRotate.setPosition(0.3);
    }

    public void RotateClawToPos(double pos) {
        intakeRotate.setPosition(pos);
    }

    public void PivotClawDown() {
        intakePivot.setPosition(0.0);
    }

    public void PivotClawUp() {
        intakePivot.setPosition(0.85);
    }

    public void PivotClawMid() {
        intakePivot.setPosition(0.35);
    }

    public void ExtendSlidesFull() {
        intakeSlidesLeft.setPosition(0.7);
    }

    public void RetractSlidesFull() {
        intakeSlidesLeft.setPosition(0.3);
    }

    public void ExtendSlidesToPos(double pos) {
        intakeSlidesLeft.setPosition(pos);
    }
}