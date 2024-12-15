package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.LimeLight.Megatag2Relocalizer;


@TeleOp(name = "Megatag2RelocalizerTest", group = "Sensor")
public class Megatag2RelocalizerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Megatag2Relocalizer relocalizer = new Megatag2Relocalizer(hardwareMap);

        telemetry.setMsTransmissionInterval(11);

        if (relocalizer.isLimelightConnected()) {
            telemetry.addData(">", "Limelight Connected");
        } else {
            telemetry.addData(">", "Limelight Not Connected");
        }
        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Looking!");

            Pose3D botpose = relocalizer.getBotPose();
            if (botpose != null) {
                telemetry.addData("Botpose", botpose.toString());
            }
            telemetry.update();
        }
    }
}