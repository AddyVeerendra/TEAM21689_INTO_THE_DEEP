package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareClasses.DepositAssembly;
import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeAssembly;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;

@TeleOp(name = "QUALIFIER V2 Robot TeleOp")
public class NewTeleopQual extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private IntakeAssembly intakeAssembly;
    private DepositAssembly depositAssembly;
    private LinearSlide linearSlides;

    // Toggles
    private boolean clawOpen = false;
    private boolean clawRotated = true;
    private boolean intakeSequenceToggle = true;
    private boolean depositSampleToggle = true;

    // Button State Tracking
    private boolean rightBumperPressed = false;
    private boolean yPressed = false;
    private boolean aPressed = false;
    private boolean dpadLeftPressed = false;

    @Override
    public void runOpMode() {
        // Initialize Drive Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        String[] motorNames = {"slideMotorLeft", "slideMotorRight"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE};
        linearSlides = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, 0, 37.5);

        intakeAssembly = new IntakeAssembly(hardwareMap);
        depositAssembly = new DepositAssembly(hardwareMap);

        // Initial positions
        depositAssembly.OpenOuttakeClaw();
        linearSlides.moveSlidesToPositionInches(0);
        depositAssembly.TransferSample();
        intakeAssembly.RotateClaw0();
        intakeAssembly.PivotClawUp();
        intakeAssembly.ExtendSlidesToPos(0.40);
        intakeAssembly.OpenClaw();

        while (opModeInInit() && !isStopRequested()) {
            // Waiting for start
        }

        waitForStart();

        while (opModeIsActive()) {
            // Drive control
            double speedMultiplier = 1 - (0.7 * gamepad2.left_trigger);
            double y = -gamepad2.left_stick_y * speedMultiplier;
            double x = gamepad2.left_stick_x * 1.1 * speedMultiplier;
            double rx = gamepad2.right_stick_x * speedMultiplier;

            double frontLeftPower = clipPower(y + x + rx);
            double backLeftPower = clipPower(y - x + rx);
            double frontRightPower = clipPower(y - x - rx);
            double backRightPower = clipPower(y + x - rx);

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            // Claw rotation toggle on Y
            if (gamepad1.y && !yPressed) {
                clawRotated = !clawRotated;
                if (clawRotated) {
                    intakeAssembly.RotateClaw0();
                } else {
                    intakeAssembly.RotateClaw90();
                }
                yPressed = true;
            } else if (!gamepad1.y) {
                yPressed = false;
            }

            if (gamepad1.right_bumper && !rightBumperPressed) {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    depositAssembly.OpenOuttakeClaw();
                    intakeAssembly.CloseClaw();
                } else {
                    depositAssembly.CloseOuttakeClaw();
                    intakeAssembly.OpenClaw();
                }
                rightBumperPressed = true;
            } else if (!gamepad1.right_bumper) {
                rightBumperPressed = false;
            }

            // Pivot claw down on X
            if (gamepad1.x) {
                intakeAssembly.PivotClawDown();
            }

            // Unlock intake on left trigger fully pressed
            if (gamepad1.left_trigger > 0.9) {
                intakeAssembly.UnlockIntake();
            }

            // Open outtake claw on gamepad2 right bumper
            if (gamepad2.right_bumper) {
                depositAssembly.OpenOuttakeClaw();
            }

            // Intake sequence toggle on A
            if (gamepad1.a && !aPressed) {
                intakeSequenceToggle = !intakeSequenceToggle;
                startIntakeSequence(intakeSequenceToggle);
                aPressed = true;
            } else if (!gamepad1.a) {
                aPressed = false;
            }

            // Deposit sample toggle on dpad_left
            if (gamepad1.dpad_left && !dpadLeftPressed) {
                depositSampleToggle = !depositSampleToggle;
                startDepositSequence(depositSampleToggle);
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }

            // Update FSMs
            updateIntakeSequence();
            updateDepositSequence();

            // Update linear slides
            linearSlides.update();
        }
    }

    private double clipPower(double power) {
        return Math.max(-0.7, Math.min(0.7, power));
    }

    // --- Intake Sequence FSM ---
    private enum IntakeSequenceState {
        IDLE,
        // Sequence when intakeSequenceToggle == true
        START_1,
        WAIT_CLOSE_CLAW,
        ROTATE_UP,
        WAIT_ROTATE_UP,
        EXTEND_SLIDES,
        WAIT_EXTEND_SLIDES,
        CLOSE_OUTTAKE_CLAW,
        WAIT_OUTTAKE_CLAW,
        OPEN_CLAW_FINISH,
        DONE_1,

        // Sequence when intakeSequenceToggle == false
        START_2,
        WAIT_UNLOCK_INTAKE,
        EXTEND_SLIDES_FULL,
        SET_PIVOT_MID,
        OPEN_CLAW_2,
        DONE_2
    }

    private IntakeSequenceState intakeState = IntakeSequenceState.IDLE;
    private double intakeStateStartTime;

    private void startIntakeSequence(boolean sequenceOne) {
        if (sequenceOne) {
            // Sequence 1 start
            intakeState = IntakeSequenceState.START_1;
        } else {
            // Sequence 2 start
            intakeState = IntakeSequenceState.START_2;
        }
        intakeStateStartTime = getRuntime();
    }

    private void updateIntakeSequence() {
        double elapsed = getRuntime() - intakeStateStartTime;
        switch (intakeState) {
            case IDLE:
                // Do nothing until triggered
                break;

            // -------- Sequence 1 (intakeSequenceToggle == true) --------
            case START_1:
                intakeAssembly.CloseClaw();
                depositAssembly.OpenOuttakeClaw();
                intakeState = IntakeSequenceState.WAIT_CLOSE_CLAW;
                intakeStateStartTime = getRuntime();
                break;

            case WAIT_CLOSE_CLAW:
                if (elapsed > 0.15) {
                    intakeAssembly.RotateClaw0();
                    intakeAssembly.PivotClawUp();
                    intakeState = IntakeSequenceState.ROTATE_UP;
                    intakeStateStartTime = getRuntime();
                }
                break;

            case ROTATE_UP:
                if (elapsed > 0.4) {
                    intakeAssembly.ExtendSlidesToPos(0.39);
                    intakeState = IntakeSequenceState.EXTEND_SLIDES;
                    intakeStateStartTime = getRuntime();
                }
                break;

            case EXTEND_SLIDES:
                if (elapsed > 0.65) {
                    depositAssembly.CloseOuttakeClaw();
                    intakeAssembly.LockIntake();
                    intakeState = IntakeSequenceState.CLOSE_OUTTAKE_CLAW;
                    intakeStateStartTime = getRuntime();
                }
                break;

            case CLOSE_OUTTAKE_CLAW:
                if (elapsed > 0.15) {
                    intakeAssembly.OpenClaw();
                    intakeState = IntakeSequenceState.DONE_1;
                }
                break;

            case DONE_1:
                // Sequence 1 done
                intakeState = IntakeSequenceState.IDLE;
                break;

            // -------- Sequence 2 (intakeSequenceToggle == false) --------
            case START_2:
                intakeAssembly.UnlockIntake();
                intakeState = IntakeSequenceState.WAIT_UNLOCK_INTAKE;
                intakeStateStartTime = getRuntime();
                break;

            case WAIT_UNLOCK_INTAKE:
                if (elapsed > 0.15) {
                    intakeAssembly.ExtendSlidesFull();
                    intakeState = IntakeSequenceState.EXTEND_SLIDES_FULL;
                    intakeStateStartTime = getRuntime();
                }
                break;

            case EXTEND_SLIDES_FULL:
                // No explicit wait needed here unless you want a delay
                intakeAssembly.PivotClawMid();
                intakeState = IntakeSequenceState.SET_PIVOT_MID;
                intakeStateStartTime = getRuntime();
                break;

            case SET_PIVOT_MID:
                // Could add a small delay if needed; if not, move on immediately
                intakeAssembly.OpenClaw();
                intakeState = IntakeSequenceState.OPEN_CLAW_2;
                intakeStateStartTime = getRuntime();
                break;

            case OPEN_CLAW_2:
                // Sequence 2 done
                intakeState = IntakeSequenceState.DONE_2;
                break;

            case DONE_2:
                intakeState = IntakeSequenceState.IDLE;
                break;
        }
    }

    // --- Deposit Sequence FSM (Triggered by dpad_left) ---
    private enum DepositSequenceState {
        IDLE,
        // depositSampleToggle == true
        OPEN_OUTTAKE,
        WAIT_OUTTAKE_OPEN,
        RESET_SLIDES,
        TRANSFER_SAMPLE,
        DONE_TRUE,

        // depositSampleToggle == false
        EXTEND_SLIDES_SCORE,
        SCORE_SAMPLE,
        DONE_FALSE
    }

    private DepositSequenceState depositState = DepositSequenceState.IDLE;
    private double depositStateStartTime;

    private void startDepositSequence(boolean sequenceOne) {
        if (sequenceOne) {
            depositState = DepositSequenceState.OPEN_OUTTAKE;
        } else {
            depositState = DepositSequenceState.EXTEND_SLIDES_SCORE;
        }
        depositStateStartTime = getRuntime();
    }

    private void updateDepositSequence() {
        double elapsed = getRuntime() - depositStateStartTime;
        switch (depositState) {
            case IDLE:
                // Do nothing until triggered
                break;

            case OPEN_OUTTAKE:
                depositAssembly.OpenOuttakeClaw();
                depositState = DepositSequenceState.WAIT_OUTTAKE_OPEN;
                depositStateStartTime = getRuntime();
                break;

            case WAIT_OUTTAKE_OPEN:
                if (elapsed > 0.15) {
                    linearSlides.moveSlidesToPositionInches(0);
                    depositAssembly.TransferSample();
                    depositState = DepositSequenceState.DONE_TRUE;
                }
                break;

            case DONE_TRUE:
                // Sequence done
                depositState = DepositSequenceState.IDLE;
                break;

            case EXTEND_SLIDES_SCORE:
                linearSlides.moveSlidesToPositionInches(35);
                depositAssembly.ScoreSample();
                depositState = DepositSequenceState.DONE_FALSE;
                break;

            case DONE_FALSE:
                // Sequence done
                depositState = DepositSequenceState.IDLE;
                break;
        }
    }
}