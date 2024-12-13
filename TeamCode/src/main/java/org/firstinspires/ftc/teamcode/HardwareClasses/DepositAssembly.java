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
        outtakePivotLeft.setPosition(0.9);
        outtakePivotRight.setPosition(0.9);
        outtakeClawPivot.setPosition(1);
    }

    public void ScoreSpecimen() {
        outtakePivotLeft.setPosition(0.4);
        outtakePivotRight.setPosition(0.4);
        outtakeClawPivot.setPosition(1);
    }

    public void InitSampleAuto() {
        outtakePivotLeft.setPosition(0.7);
        outtakePivotRight.setPosition(0.7);
        outtakeClawPivot.setPosition(1);
    }

    public void TransferSample() {
        outtakePivotLeft.setPosition(0.4);
        outtakePivotRight.setPosition(0.4);
        outtakeClawPivot.setPosition(0.87);
        OpenOuttakeClaw();
    }
}