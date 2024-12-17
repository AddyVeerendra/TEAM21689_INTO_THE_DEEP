package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DepositAssembly {
    private Servo outtakePivotLeft;
    private Servo outtakePivotRight;
    private Servo outtakeClaw;
    private Servo outtakeClawPivot;
    private Servo outtakeClawRotate;

    public DepositAssembly(HardwareMap hardwareMap) {
        outtakePivotLeft = hardwareMap.get(Servo.class, "outtakePivotLeft");
        outtakePivotLeft.setDirection(Servo.Direction.REVERSE);
        outtakePivotRight = hardwareMap.get(Servo.class, "outtakePivotRight");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        outtakeClawPivot = hardwareMap.get(Servo.class, "outtakeClawPivot");
        outtakeClawRotate = hardwareMap.get(Servo.class, "outtakeClawRotate");
    }

    public void RotateOuttakeClaw0() {
        outtakeClawRotate.setPosition(0.3);
    }

    public void RotateOuttakeClaw180() {
        outtakeClawRotate.setPosition(0.9);
    }

    public void CloseOuttakeClaw() {
        outtakeClaw.setPosition(1);
    }

    public void OpenOuttakeClaw() {
        outtakeClaw.setPosition(0.75);
    }

    public void ScoreSample() {
        outtakePivotLeft.setPosition(0.15);
        outtakePivotRight.setPosition(0.15);
        outtakeClawPivot.setPosition(0.9);
        RotateOuttakeClaw0();
    }

    public void GrabSpecimen() {
        outtakePivotLeft.setPosition(0.0);
        outtakePivotRight.setPosition(0.0);
        outtakeClawPivot.setPosition(1);
        RotateOuttakeClaw180();
    }

    public void ScoreSpecimen() {
        outtakePivotLeft.setPosition(0.65);
        outtakePivotRight.setPosition(0.65);
        outtakeClawPivot.setPosition(0.9);
        RotateOuttakeClaw0();
    }

    public void TransferSample() {
        outtakePivotLeft.setPosition(0.7);
        outtakePivotRight.setPosition(0.70);
        outtakeClawPivot.setPosition(1);
        OpenOuttakeClaw();
        RotateOuttakeClaw0();
    }
}