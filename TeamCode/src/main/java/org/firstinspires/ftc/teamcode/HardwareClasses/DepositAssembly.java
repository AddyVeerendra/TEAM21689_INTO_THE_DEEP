package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DepositAssembly {
    private Servo outtakePivotLeft;
    private Servo outtakePivotRight;
    private Servo outtakeClaw;
    private Servo outtakeClawPivot;

    public DepositAssembly(HardwareMap hardwareMap) {
        outtakePivotLeft = hardwareMap.get(Servo.class, "outtakePivotLeft");
        outtakePivotLeft.setDirection(Servo.Direction.REVERSE);
        outtakePivotRight = hardwareMap.get(Servo.class, "outtakePivotRight");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        outtakeClawPivot = hardwareMap.get(Servo.class, "outtakeClawPivot");
    }

    public void CloseOuttakeClaw() {
        outtakeClaw.setPosition(1);
    }

    public void OpenOuttakeClaw() {
        outtakeClaw.setPosition(0.75);
    }

    public void ScoreSample() {
        outtakePivotLeft.setPosition(0.08);
        outtakePivotRight.setPosition(0.08);
        outtakeClawPivot.setPosition(0.9);
    }

    public void GrabSpecimen() {
        outtakePivotLeft.setPosition(0.0);
        outtakePivotRight.setPosition(0.0);
        outtakeClawPivot.setPosition(1);
    }

    public void ScoreSpecimen() {
        outtakePivotLeft.setPosition(0.65);
        outtakePivotRight.setPosition(0.65);
        outtakeClawPivot.setPosition(0.9);
    }

    public void InitSampleAuto() {
        outtakePivotLeft.setPosition(0.33);
        outtakePivotRight.setPosition(0.33);
        outtakeClawPivot.setPosition(1);
    }

    public void TransferSample() {
        outtakePivotLeft.setPosition(0.60);
        outtakePivotRight.setPosition(0.60);
        outtakeClawPivot.setPosition(0.83);
        OpenOuttakeClaw();
    }
}