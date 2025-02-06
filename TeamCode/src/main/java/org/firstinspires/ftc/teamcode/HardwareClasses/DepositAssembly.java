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
        outtakeClawRotate.setPosition(1);
    }

    public void CloseOuttakeClaw() {
        outtakeClaw.setPosition(1);
    }

    public void OpenOuttakeClaw() {
        outtakeClaw.setPosition(0.70);
    }

    public void ScoreSample() {
        outtakePivotLeft.setPosition(0.27);
        outtakePivotRight.setPosition(0.27);
        outtakeClawPivot.setPosition(0.9);
        RotateOuttakeClaw0();
    }

    public void ScoreSampleLow() {
        outtakePivotLeft.setPosition(0.15);
        outtakePivotRight.setPosition(0.15);
        outtakeClawPivot.setPosition(0.9);
        RotateOuttakeClaw0();
    }

    public void ScoreSampleFront() {
        outtakePivotLeft.setPosition(0.65);
        outtakePivotRight.setPosition(0.65);
        outtakeClawPivot.setPosition(0.9);
        RotateOuttakeClaw180();
    }

    public void GrabSampleFloor() {
        outtakePivotLeft.setPosition(0.03);
        outtakePivotRight.setPosition(0.03);
        outtakeClawPivot.setPosition(0.65);
        RotateOuttakeClaw0();
    }

    public void ScoreSpecimen() {
        outtakePivotLeft.setPosition(0.4);
        outtakePivotRight.setPosition(0.4);
        outtakeClawPivot.setPosition(0.5);
        RotateOuttakeClaw180();
    }
    public void GrabSpecimen() {
        outtakePivotLeft.setPosition(0.81);
        outtakePivotRight.setPosition(0.81);
        outtakeClawPivot.setPosition(0.85);
        RotateOuttakeClaw0();
    }

    public void TransferSample() {
        outtakePivotLeft.setPosition(0.83);
        outtakePivotRight.setPosition(0.83);
        outtakeClawPivot.setPosition(0.99);
        OpenOuttakeClaw();
        RotateOuttakeClaw0();
    }

    public void TransferSampleLow() {
        outtakePivotLeft.setPosition(0.85);
        outtakePivotRight.setPosition(0.85);
        outtakeClawPivot.setPosition(0.99);
        OpenOuttakeClaw();
        RotateOuttakeClaw0();
    }

    public void Hang() {
        outtakePivotLeft.setPosition(0.58);
        outtakePivotRight.setPosition(0.58);
        outtakeClawPivot.setPosition(0.4);
        RotateOuttakeClaw180();
    }

    public void HangAuto() {
        outtakePivotLeft.setPosition(0.33);
        outtakePivotRight.setPosition(0.33);
        outtakeClawPivot.setPosition(0.4);
        RotateOuttakeClaw180();
    }

    public void GrabSampleFloor90() {
        outtakePivotLeft.setPosition(0.03);
        outtakePivotRight.setPosition(0.03);
        outtakeClawPivot.setPosition(0.7);
        outtakeClawRotate.setPosition(0.7);
    }
}
