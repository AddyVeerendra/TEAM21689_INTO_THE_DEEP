package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeAssemblyIntake {
    private CRServo intakeRotationServo;
    private Servo intakePitchServo;
    private Servo intakeLock;

    private Pivot intakeSlides;

    private ColorV3 colorV3;

    public IntakeAssemblyIntake(HardwareMap hardwareMap) {
        intakeRotationServo = hardwareMap.get(CRServo.class, "intakePivot");
        intakePitchServo = hardwareMap.get(Servo.class, "intakeSlidesLeft");
        intakeLock = hardwareMap.get(Servo.class, "intakeLock");
        intakeSlides = new Pivot(hardwareMap);
        colorV3 = new ColorV3(hardwareMap);
    }

    public ColorV3 getColorV3() {
        return colorV3;
    }

    public void LockIntake() {
        intakeLock.setPosition(0.0);
    }

    public void UnlockIntake() {
        intakeLock.setPosition(0.26);
    }

    public void IntakeOn() {
        intakeRotationServo.setPower(-1);
    }

    public void IntakeOff() {
        intakeRotationServo.setPower(0);
    }

    public void IntakeReverse() {
        intakeRotationServo.setPower(1);
    }

    public void PivotIntakeDown() {
        intakePitchServo.setPosition(0.78);
    }

    public void PivotIntakeUp() {
        intakePitchServo.setPosition(0.15);
    }

    public void PivotIntakeTransfer() {
        intakePitchServo.setPosition(0.2);
    }

    public void PivotIntakeMid() {
        intakePitchServo.setPosition(0.4);
    }

    public void ExtendSlidesFull() {
        intakeSlides.movePivotToAngle(74);
    }

    public void RetractSlidesFull() {
        intakeSlides.movePivotToAngle(0);
    }

    public void ExtendSlidesToPos(double pos) {
        intakeSlides.movePivotToAngle(pos);
    }

    public void update() {
        intakeSlides.update();
    }
}