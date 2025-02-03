package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareClasses.DepositAssembly;
import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeAssemblyClaw;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;

@TeleOp(name = "AAAAHH SIGMA TELEOP STATES ADDY THINGY")
@Disabled
public class StatesTeleopFrontWheel extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private IntakeAssemblyClaw intakeAssembly;
    private DepositAssembly depositAssembly;
    private LinearSlide linearSlides;

    // Toggles
    private boolean clawOpen = false;
    private boolean clawRotated = true;
    private boolean intakeSequenceToggle = true;
    private boolean depositSampleToggle = true;
    private boolean depositSpecimenToggle = false;

    // Button State Tracking
    private boolean rightBumperPressed = false;
    private boolean yPressed = true;
    private boolean aPressed = false;
    private boolean dpadLeftPressed = false;
    public boolean lockDpad = false;
    private boolean dpadRightPressed = false;

    // --- New variables for button B ---
    private boolean bInputPressed = false;
    // This toggle flips each time B is pressed. True = extend sequence; false = retract sequence.
    private boolean intakeBToggle = false;

    public int times = 0;
    private boolean frontWheelTurn = false;

    private float frontWheelRadius = 2.0f;

    private float backWheelRadius = 3.0f;

    @Override
    public void runOpMode() {
        // Initialize Drive motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        String[] motorNames = {"slideMotorLeft", "slideMotorRight"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE};
        linearSlides = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, -37.5, 37.5);

        intakeAssembly = new IntakeAssemblyClaw(hardwareMap);
        depositAssembly = new DepositAssembly(hardwareMap);

        // Initial positions
        intakeAssembly.IntakeFlickerVertical();
        intakeAssembly.UnlockIntake();
        depositAssembly.OpenOuttakeClaw();
        depositAssembly.Hang();
        intakeAssembly.RotateClaw0();
        intakeAssembly.PivotClawUp();
        intakeAssembly.OpenClaw();
        intakeAssembly.IntakeFlickerVertical();
        intakeAssembly.setOffset(15);

        resetRuntime();

        while (opModeInInit() && !isStopRequested()) {
            if (getRuntime() > 0 && times == 0) {
                intakeAssembly.ExtendSlidesToPos(-95);
                times++;
            } else if (getRuntime() > 0.7 && times == 1) {
                intakeAssembly.zeroSlide();
                times++;
            } else if (getRuntime() > 0.9 && times == 2) {
                intakeAssembly.ExtendSlidesToPos(20);
                linearSlides.moveSlidesToPositionInches(0);
                gamepad1.rumble(200);
                gamepad2.rumble(200);
                times++;
            }

            intakeAssembly.update();
            linearSlides.update();
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

            if (gamepad2.left_bumper) {
                if (frontWheelTurn) {
                    frontWheelTurn = false;
                } else {
                    frontLeftPower *= (frontWheelRadius / backWheelRadius);
                    frontRightPower *= (frontWheelRadius / backWheelRadius);
                    frontWheelTurn = true;
                }
            }
            
            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            if (gamepad1.options) {
                linearSlides.moveSlidesToPositionInches(-20);
            } else if (gamepad1.share) {
                linearSlides.zeroSlides();
                linearSlides.moveSlidesToPositionInches(0);
            }

            if (gamepad2.dpad_up) {
                linearSlides.moveSlidesToPositionInches(33);
            }

            if (gamepad2.options) {
                intakeAssembly.ExtendSlidesToPos(-40);
            } else if (gamepad2.share) {
                intakeAssembly.zeroSlide();
                intakeAssembly.update();
                intakeAssembly.ExtendSlidesToPos(20);
                linearSlides.moveSlidesToPositionInches(15);
            }

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

            if (gamepad1.dpad_up) {
                depositAssembly.Hang();
                intakeAssembly.UnlockIntake();
                intakeAssembly.RetractSlidesFull();
                linearSlides.moveSlidesToPositionInches(15);
            } else if (gamepad1.dpad_down) {
                linearSlides.moveSlidesToPositionInches(0);
                intakeAssembly.LockIntake();
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

            // Unlock intake on right trigger fully pressed
            if (gamepad1.right_trigger > 0.9) {
                depositAssembly.ScoreSampleLow();
            }

            if (gamepad1.left_trigger > 0.9) {
                linearSlides.moveSlidesToPositionInches(23);
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

            // --- New: Intake sequence toggle on B ---
            if (gamepad1.b && !bInputPressed) {
                intakeBToggle = !intakeBToggle;
                startIntakeBSequence(intakeBToggle);
                bInputPressed = true;
            } else if (!gamepad1.b) {
                bInputPressed = false;
            }

            // Deposit sample toggle on dpad_left
            if (gamepad1.dpad_left && !dpadLeftPressed && !lockDpad) {
                depositSampleToggle = !depositSampleToggle;
                startSampleDepositSequence(depositSampleToggle);
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }

            if (gamepad1.dpad_right && !dpadRightPressed) {
                depositSpecimenToggle = !depositSpecimenToggle;
                startSpecimenDepositSequence(depositSpecimenToggle);
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }

            // Update FSMs
            updateIntakeSequence();
            updateDepositSequence();
            updateIntakeBSequence();

            // Update linear slides and intake assembly
            linearSlides.update();
            intakeAssembly.update();
        }
    }

    private double clipPower(double power) {
        return Math.max(-1, Math.min(1, power));
    }

    // --- Intake Sequence FSM (Triggered by A) ---
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
            intakeState = IntakeSequenceState.START_1;
        } else {
            intakeState = IntakeSequenceState.START_2;
        }
        intakeStateStartTime = getRuntime();
    }

    private void updateIntakeSequence() {
        double elapsed = getRuntime() - intakeStateStartTime;
        switch (intakeState) {
            case IDLE:
                break;
            case START_1:
                lockDpad = true;
                intakeAssembly.CloseClaw();
                depositAssembly.OpenOuttakeClaw();
                depositAssembly.TransferSample();
                intakeState = IntakeSequenceState.WAIT_CLOSE_CLAW;
                linearSlides.moveSlidesToPositionInches(0);
                intakeStateStartTime = getRuntime();
                intakeAssembly.IntakeFlickerVertical();
                break;
            case WAIT_CLOSE_CLAW:
                if (elapsed > 0.25) {
                    intakeAssembly.RotateClaw0();
                    intakeAssembly.PivotClawUp();
                    intakeState = IntakeSequenceState.ROTATE_UP;
                    intakeStateStartTime = getRuntime();
                }
                break;
            case ROTATE_UP:
                if (elapsed > 0.4) {
                    intakeAssembly.ExtendSlidesToPos(21);
                    intakeState = IntakeSequenceState.EXTEND_SLIDES;
                    intakeStateStartTime = getRuntime();
                }
                break;
            case EXTEND_SLIDES:
                if (elapsed > 0.4) {
                    depositAssembly.CloseOuttakeClaw();
                    intakeAssembly.LockIntake();
                    intakeState = IntakeSequenceState.CLOSE_OUTTAKE_CLAW;
                    intakeStateStartTime = getRuntime();
                }
                break;
            case CLOSE_OUTTAKE_CLAW:
                if (elapsed > 0.15) {
                    gamepad2.rumble(200);
                    intakeAssembly.ExtendSlidesToPos(22);
                    intakeAssembly.OpenClaw();
                    intakeState = IntakeSequenceState.DONE_1;
                }
                break;
            case DONE_1:
                lockDpad = false;
                intakeState = IntakeSequenceState.IDLE;
                break;
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
                intakeAssembly.PivotClawMid();
                intakeAssembly.IntakeFlickerUp();
                intakeState = IntakeSequenceState.SET_PIVOT_MID;
                intakeStateStartTime = getRuntime();
                break;
            case SET_PIVOT_MID:
                intakeAssembly.OpenClaw();
                intakeState = IntakeSequenceState.OPEN_CLAW_2;
                intakeStateStartTime = getRuntime();
                break;
            case OPEN_CLAW_2:
                intakeState = IntakeSequenceState.DONE_2;
                break;
            case DONE_2:
                intakeState = IntakeSequenceState.IDLE;
                break;
        }
    }

    // --- Deposit Sequence FSM (Triggered by dpad_left and dpad_right) ---
    private enum DepositSequenceState {
        IDLE,
        // depositSampleToggle == true
        OPEN_OUTTAKE_SAMPLE,
        WAIT_OUTTAKE_OPEN_SAMPLE,
        DONE_TRUE_SAMPLE,
        // depositSampleToggle == false
        EXTEND_SLIDES_SCORE_SAMPLE,
        DONE_FALSE_SAMPLE,
        // depositSpecimenToggle == true
        RETRACT_SLIDES_SPECIMEN_SCORE,
        RETRACT_SLIDES_SPECIMEN_GRAB,
        DONE_TRUE_SPECIMEN,
        // depositSpecimenToggle == false
        CLOSE_OUTTAKE_SPECIMEN,
        WAIT_OUTTAKE_CLOSE_SPECIMEN,
        DONE_FALSE_SPECIMEN
    }

    private DepositSequenceState depositState = DepositSequenceState.IDLE;
    private double depositStateStartTime;

    private void startSampleDepositSequence(boolean sequenceOne) {
        if (sequenceOne) {
            depositState = DepositSequenceState.OPEN_OUTTAKE_SAMPLE;
        } else {
            depositState = DepositSequenceState.EXTEND_SLIDES_SCORE_SAMPLE;
        }
        depositStateStartTime = getRuntime();
    }

    private void startSpecimenDepositSequence(boolean sequenceOne) {
        if (sequenceOne) {
            depositState = DepositSequenceState.RETRACT_SLIDES_SPECIMEN_SCORE;
        } else {
            depositState = DepositSequenceState.CLOSE_OUTTAKE_SPECIMEN;
        }
        depositStateStartTime = getRuntime();
    }

    private void updateDepositSequence() {
        double elapsed = getRuntime() - depositStateStartTime;
        switch (depositState) {
            case IDLE:
                break;
            case OPEN_OUTTAKE_SAMPLE:
                depositAssembly.OpenOuttakeClaw();
                depositState = DepositSequenceState.WAIT_OUTTAKE_OPEN_SAMPLE;
                depositStateStartTime = getRuntime();
                break;
            case WAIT_OUTTAKE_OPEN_SAMPLE:
                if (elapsed > 0.15) {
                    linearSlides.moveSlidesToPositionInches(0);
                    depositAssembly.TransferSample();
                    depositState = DepositSequenceState.DONE_TRUE_SAMPLE;
                }
                break;
            case DONE_TRUE_SAMPLE:
                depositState = DepositSequenceState.IDLE;
                break;
            case EXTEND_SLIDES_SCORE_SAMPLE:
                linearSlides.moveSlidesToPositionInches(31);
                depositAssembly.ScoreSample();
                depositState = DepositSequenceState.DONE_FALSE_SAMPLE;
                break;
            case DONE_FALSE_SAMPLE:
                depositState = DepositSequenceState.IDLE;
                break;
            case RETRACT_SLIDES_SPECIMEN_SCORE:
                linearSlides.setKP(0.005);
                linearSlides.moveSlidesToPositionInches(5);
                intakeAssembly.ExtendSlidesToPos(10);
                intakeAssembly.UnlockIntake();
                depositState = DepositSequenceState.RETRACT_SLIDES_SPECIMEN_GRAB;
                depositStateStartTime = getRuntime();
                break;
            case RETRACT_SLIDES_SPECIMEN_GRAB:
                if (elapsed > 0.5) {
                    linearSlides.setKP(0.005);
                    linearSlides.moveSlidesToPositionInches(2);
                    depositAssembly.GrabSpecimen();
                    depositAssembly.OpenOuttakeClaw();
                    depositState = DepositSequenceState.DONE_TRUE_SPECIMEN;
                }
                break;
            case DONE_TRUE_SPECIMEN:
                if (elapsed > 1.5) {
                    linearSlides.setKP(0.005);
                    linearSlides.moveSlidesToPositionInches(0);
                    depositAssembly.GrabSpecimen();
                    depositAssembly.OpenOuttakeClaw();
                    depositState = DepositSequenceState.IDLE;
                }
                break;
            case CLOSE_OUTTAKE_SPECIMEN:
                depositAssembly.CloseOuttakeClaw();
                depositState = DepositSequenceState.WAIT_OUTTAKE_CLOSE_SPECIMEN;
                depositStateStartTime = getRuntime();
                break;
            case WAIT_OUTTAKE_CLOSE_SPECIMEN:
                if (elapsed > 0.15) {
                    linearSlides.moveSlidesToPositionInches(13);
                    intakeAssembly.UnlockIntake();
                    intakeAssembly.ExtendSlidesToPos(10);
                    depositAssembly.ScoreSpecimen();
                    depositState = DepositSequenceState.DONE_FALSE_SPECIMEN;
                }
                break;
            case DONE_FALSE_SPECIMEN:
                depositState = DepositSequenceState.IDLE;
                break;
        }
    }

    // --- New: Intake B Sequence FSM (Triggered by button B) ---
    private enum IntakeBSequenceState {
        IDLE,
        START_B,
        WAIT_CLOSE_CLAW_B,
        ROTATE_UP_B,
        EXTEND_SLIDES_B,
        RETRACT_B,
        DONE_B
    }

    private IntakeBSequenceState intakeBState = IntakeBSequenceState.IDLE;
    private double intakeBStateStartTime;

    /**
     * Starts the B sequence.
     * @param extend if true, run the extension sequence; if false, run the retraction.
     */
    private void startIntakeBSequence(boolean extend) {
        intakeBStateStartTime = getRuntime();
        if (extend) {
            intakeBState = IntakeBSequenceState.START_B;
        } else {
            intakeBState = IntakeBSequenceState.RETRACT_B;
        }
    }

    private void updateIntakeBSequence() {
        double elapsed = getRuntime() - intakeBStateStartTime;
        switch (intakeBState) {
            case IDLE:
                break;
            case START_B:
                lockDpad = true;
                intakeAssembly.CloseClaw();
                depositAssembly.OpenOuttakeClaw();
                // Skip transferring the sample!
                linearSlides.moveSlidesToPositionInches(0);
                intakeAssembly.IntakeFlickerVertical();
                intakeBState = IntakeBSequenceState.WAIT_CLOSE_CLAW_B;
                intakeBStateStartTime = getRuntime();
                break;
            case WAIT_CLOSE_CLAW_B:
                if (elapsed > 0.25) {
                    intakeAssembly.RotateClaw0();
                    intakeAssembly.PivotClawUp();
                    intakeBState = IntakeBSequenceState.ROTATE_UP_B;
                    intakeBStateStartTime = getRuntime();
                }
                break;
            case ROTATE_UP_B:
                if (elapsed > 0.4) {
                    intakeAssembly.ExtendSlidesToPos(21);
                    intakeBState = IntakeBSequenceState.EXTEND_SLIDES_B;
                    intakeBStateStartTime = getRuntime();
                }
                break;
            case EXTEND_SLIDES_B:
                if (elapsed > 0.4) {
                    // Extension complete – additional commands can be added here if needed.
                    intakeBState = IntakeBSequenceState.DONE_B;
                }
                break;
            case RETRACT_B:
                // For retraction: pivot the claw up and retract the slides.
                intakeAssembly.PivotClawUp();
                linearSlides.moveSlidesToPositionInches(0);
                if (elapsed > 0.3) {
                    intakeBState = IntakeBSequenceState.DONE_B;
                    intakeBStateStartTime = getRuntime();
                }
                break;
            case DONE_B:
                lockDpad = false;
                intakeBState = IntakeBSequenceState.IDLE;
                break;
        }
    }
}
