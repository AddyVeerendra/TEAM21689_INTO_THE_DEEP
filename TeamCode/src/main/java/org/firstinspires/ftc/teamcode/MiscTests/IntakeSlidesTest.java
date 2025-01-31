package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeAssemblyClaw;

@TeleOp(name = "Intake Slides Test", group = "TeleOp")
@Disabled
public class IntakeSlidesTest extends LinearOpMode {

    private IntakeAssemblyClaw intakeAssembly;

    @Override
    public void runOpMode() {
        // Initialize the servo
        intakeAssembly = new IntakeAssemblyClaw(hardwareMap);

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                intakeAssembly.ExtendSlidesFull();
            } else if (gamepad1.dpad_right) {
                intakeAssembly.RetractSlidesFull();
            }
            intakeAssembly.update();
        }
    }
}