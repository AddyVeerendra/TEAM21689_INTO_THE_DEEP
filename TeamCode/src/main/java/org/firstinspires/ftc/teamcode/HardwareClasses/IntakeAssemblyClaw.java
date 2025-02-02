package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeAssemblyClaw {
    private Servo intakePivot;
    private Servo intakeRotate;
    private Servo intakeClaw;
    private Servo intakeLock;

    private Servo intakeFlicker;

    private Pivot intakeSlides;

    public IntakeAssemblyClaw(HardwareMap hardwareMap) {
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        intakePivot.setDirection(Servo.Direction.REVERSE);
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeLock = hardwareMap.get(Servo.class, "intakeLock");
        intakeFlicker = hardwareMap.get(Servo.class, "intakeFlicker");
        intakeSlides = new Pivot(hardwareMap);
    }

    public void IntakeFlickerDown() {
        intakeFlicker.setPosition(0.01);
    }

    public void IntakeFlickerUpMid() {
        intakeFlicker.setPosition(0.3);
    }

    public void IntakeFlickerUp() {
        intakeFlicker.setPosition(0.82);
    }

    public void IntakeFlickerVertical() {
        intakeFlicker.setPosition(0.55);
    }

    public void LockIntake() {
        intakeLock.setPosition(0);
    }

    public void UnlockIntake() {
        intakeLock.setPosition(0.26);
    }

    public void OpenClaw() {
        intakeClaw.setPosition(0.66);
    }

    public void CloseClaw() {
        intakeClaw.setPosition(0.95);
    }

    public void RotateClaw0() {
        intakeRotate.setPosition(0);
    }

    public void RotateClaw135() {
        intakeRotate.setPosition(0.4);
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

    public void setOffset(double set) {
        intakeSlides.setOffset(set);
    }

    public void PivotClawMid() {
        intakePivot.setPosition(0.35);
    }

    public void ExtendSlidesFull() {
        intakeSlides.movePivotToAngle(74);
    }

    public void ExtendSlidesFullAuto() {
        intakeSlides.movePivotToAngle(40);
    }

    public void RetractSlidesFull() {
        intakeSlides.movePivotToAngle(0);
    }

    public void ExtendSlidesToPos(double pos) {
        intakeSlides.movePivotToAngle(pos);
    }

    public void zeroSlide() {
        intakeSlides.zeroPivot();
    }

    public void update() {
        intakeSlides.update();
    }
}