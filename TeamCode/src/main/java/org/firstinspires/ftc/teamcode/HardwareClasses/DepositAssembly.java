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
        outtakePivotLeft.setPosition(0.0);
        outtakePivotRight.setPosition(0.0);
        outtakeClawPivot.setPosition(1);
    }

    public void GrabSpecimen() {
        outtakePivotLeft.setPosition(0.01);
        outtakePivotRight.setPosition(0.01);
        outtakeClawPivot.setPosition(1);
    }

    public void ScoreSpecimen() {
        outtakePivotLeft.setPosition(0.58);
        outtakePivotRight.setPosition(0.58);
        outtakeClawPivot.setPosition(1);
    }

    public void InitSampleAuto() {
        outtakePivotLeft.setPosition(0.3);
        outtakePivotRight.setPosition(0.3);
        outtakeClawPivot.setPosition(1);
    }

    public void TransferSample() {
        outtakePivotLeft.setPosition(0.58);
        outtakePivotRight.setPosition(0.58);
        outtakeClawPivot.setPosition(0.87);
        OpenOuttakeClaw();
    }
}