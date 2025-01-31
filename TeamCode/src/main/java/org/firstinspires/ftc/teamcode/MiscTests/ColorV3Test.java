package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareClasses.ColorV3;

@TeleOp
@Disabled
public class ColorV3Test extends LinearOpMode {
    private ColorV3 colorV3;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        colorV3 = new ColorV3(hardwareMap);

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, check proximity and detect color
        while (opModeIsActive()) {
            String proximityAndColor = colorV3.proximityAndColor();
            String color = colorV3.sampleColor();
            boolean isConnected = colorV3.isConnected();
            double proximity = colorV3.proximity();

            telemetry.addData("Proximity and Color", proximityAndColor);
            telemetry.addData("Proximity", proximity);
            telemetry.addData("Color", color);
            telemetry.addData("Is Connected", isConnected);
            telemetry.update();
        }
    }
}