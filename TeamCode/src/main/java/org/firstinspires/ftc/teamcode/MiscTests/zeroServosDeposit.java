package org.firstinspires.ftc.teamcode.MiscTests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareClasses.DepositAssembly;

@TeleOp(name = "Zero servos deposit")
@Disabled
public class zeroServosDeposit extends LinearOpMode {

    private DepositAssembly depositAssembly;

    @Override
    public void runOpMode() {
        depositAssembly = new DepositAssembly(hardwareMap);

        depositAssembly.TransferSample();

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}