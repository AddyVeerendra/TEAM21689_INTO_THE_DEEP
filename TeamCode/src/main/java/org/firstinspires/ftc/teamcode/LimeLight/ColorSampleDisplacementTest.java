package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "ColorSampleDisplacementTest", group = "Sensor")
public class ColorSampleDisplacementTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ColorSampleDisplacement colorSampleDisplacement = new ColorSampleDisplacement(hardwareMap, "yellow");

        telemetry.setMsTransmissionInterval(11);

        if (colorSampleDisplacement.isConnected()) {
            telemetry.addData(">", "Limelight Connected");
        } else {
            telemetry.addData(">", "Limelight Not Connected");
        }
        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            Pose3D botpose = colorSampleDisplacement.getBotPose();
            double displacement = colorSampleDisplacement.getDisplacementInPixels();
            double latency = colorSampleDisplacement.getLatency();
            double parseLatency = colorSampleDisplacement.getParseLatency();

            telemetry.addData("LL Latency", latency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("Botpose", botpose != null ? botpose.toString() : "null");
            telemetry.addData("Displacement (Pixels) for yellow", displacement);
            telemetry.update();
        }
    }
}