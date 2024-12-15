package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.LimeLight.ColorSampleDisplacement;

public class RedSampleAlignTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ColorSampleDisplacement colorSampleDisplacement = new ColorSampleDisplacement(hardwareMap, "red");
        telemetry.setMsTransmissionInterval(11);

        waitForStart();
    }
}
