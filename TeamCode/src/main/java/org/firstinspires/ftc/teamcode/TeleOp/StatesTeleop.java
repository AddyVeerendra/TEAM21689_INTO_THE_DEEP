package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareClasses.DepositAssembly;
import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeAssemblyClaw;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "AAAAHH SIGMA TELEOP STATES")
public class StatesTeleop extends LinearOpMode {

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

    // --- NEW B Toggle Tracking ---
    private boolean bPressed = false;
    private boolean bSequenceToggle = false; // We'll flip this each time B is pressed.

    private boolean useLowTransfer = false;
    private boolean leftBumperTogglePressed = false;

    public int times = 0;

    private double currentY = 0.0, currentX = 0.0, currentRx = 0.0;

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
        intakeAssembly.UnlockIntakeMid();
        depositAssembly.OpenOuttakeClaw();
        depositAssembly.Hang();
        intakeAssembly.RotateClaw0();
        intakeAssembly.PivotClawUp();
        intakeAssembly.OpenClaw();
        intakeAssembly.IntakeFlickerVertical();
        intakeAssembly.setOffset(15);

        resetRuntime();

        // During INIT phase
        while (opModeInInit() && !isStopRequested()) {
            if (getRuntime() > 0.3 && times == 0) {
                intakeAssembly.ExtendSlidesToPos(-95);
                times++;
            } else if (getRuntime() > 1 && times == 1) {
                intakeAssembly.zeroSlide();
                times++;
            } else if (getRuntime() > 1.2 && times == 2) {
                intakeAssembly.ExtendSlidesToPos(20);
                linearSlides.moveSlidesToPositionInches(0);
                intakeAssembly.UnlockIntake();
                gamepad1.rumble(200);
                gamepad2.rumble(200);
                gamepad1.setLedColor(255, 105, 180, 1000);
                gamepad2.setLedColor(255, 105, 180, 1000);
                times++;
            }

            intakeAssembly.update();
            linearSlides.update();
        }

        waitForStart();

        // MAIN LOOP
        while (opModeIsActive()) {
            // Drive control
            double speedMultiplier = 1 - (0.7 * gamepad2.left_trigger);

            double currentY = -gamepad2.left_stick_y * speedMultiplier; // Forward/Back
            double currentX = gamepad2.left_stick_x * 1.1 * speedMultiplier; // Strafing
            double currentRx = Range.clip(gamepad2.right_stick_x * speedMultiplier, -0.7, 0.7); // Rotation

            double frontLeftPower = clipPower(currentY + currentX + currentRx);
            double backLeftPower = clipPower(currentY - currentX + currentRx);
            double frontRightPower = clipPower(currentY - currentX - currentRx);
            double backRightPower = clipPower(currentY + currentX - currentRx);

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            // Some examples
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

            // D-pad up/down for quick moves
            if (gamepad1.dpad_up) {
                depositAssembly.Hang();
                intakeAssembly.UnlockIntake();
                intakeAssembly.RetractSlidesFull();
                linearSlides.moveSlidesToPositionInches(25);
            } else if (gamepad1.dpad_down) {
                linearSlides.moveSlidesToPositionInches(15);
                intakeAssembly.LockIntake();
            }

//            // Toggle the outtake and intake claws
//            if (gamepad1.right_bumper && !rightBumperPressed) {
//                clawOpen = !clawOpen;
//                if (clawOpen) {
//                    depositAssembly.CloseOuttakeClaw();
//                    intakeAssembly.OpenClaw();
//                } else {
//                    depositAssembly.OpenOuttakeClaw();
//                    intakeAssembly.CloseClaw();
//                }
//                rightBumperPressed = true;
//            } else if (!gamepad1.right_bumper) {
//                rightBumperPressed = false;
//            }

            // Pivot claw down on X
            if (gamepad1.x) {
                intakeAssembly.PivotClawDown();
            }

            // Some triggers for deposit
            if (gamepad1.right_trigger > 0.9) {
                depositAssembly.ScoreSampleLow();
            }

            if (gamepad1.left_trigger > 0.9) {
                linearSlides.moveSlidesToPositionInches(23);
            }

            // Open outtake claw on gamepad2 right bumper
            if (gamepad2.right_trigger > 0.8) {
                depositAssembly.Hang();
                linearSlides.moveSlidesToPositionInches(11);
            }

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
            if (gamepad1.dpad_left && !dpadLeftPressed && !lockDpad) {
                depositSampleToggle = !depositSampleToggle;
                startSampleDepositSequence(depositSampleToggle);
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }

            // Deposit specimen toggle on dpad_right
            if (gamepad1.dpad_right && !dpadRightPressed) {
                depositSpecimenToggle = !depositSpecimenToggle;
                startSpecimenDepositSequence(depositSpecimenToggle);
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }

            // --- NEW B Toggle Handling ---
            if (gamepad1.b && !bPressed) {
                bSequenceToggle = !bSequenceToggle;  // Flip between true/false
                startBIntakeSequence(bSequenceToggle);
                bPressed = true;
            } else if (!gamepad1.b) {
                bPressed = false;
            }

            // Toggle the deposit transfer mode on gamepad1 left bumper
            if (gamepad1.left_bumper && !leftBumperTogglePressed) {
                useLowTransfer = !useLowTransfer;
                leftBumperTogglePressed = true;
                gamepad1.rumble(200);
                // Optionally, add telemetry or rumble feedback here
            } else if (!gamepad1.left_bumper) {
                leftBumperTogglePressed = false;
            }

            // Update all FSMs
            updateIntakeSequence();
            updateDepositSequence();
            updateBIntakeSequence(); // <--- NEW

            // Update hardware
            linearSlides.update();
            intakeAssembly.update();
        }
    }

    private double clipPower(double power) {
        return Math.max(-1, Math.min(1, power));
    }

    // ----------------------------------------------------------------
    // ----------------- Intake Sequence FSM (A) ----------------------
    // ----------------------------------------------------------------
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
                lockDpad = true;
                intakeAssembly.CloseClaw();
                depositAssembly.OpenOuttakeClaw();
                if (useLowTransfer) {
                    depositAssembly.TransferSampleLow();
                } else {
                    depositAssembly.TransferSample();
                }
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
                    intakeAssembly.ExtendSlidesToPos(18);
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
                // Sequence 1 done
                lockDpad = false;
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
                intakeAssembly.IntakeFlickerUp();
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

    // ----------------------------------------------------------------
    // ---------------- Deposit Sequence FSM (dpad_left/right) --------
    // ----------------------------------------------------------------
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
                // Do nothing until triggered
                break;

            // depositSampleToggle == true
            case OPEN_OUTTAKE_SAMPLE:
                depositAssembly.OpenOuttakeClaw();
                depositState = DepositSequenceState.WAIT_OUTTAKE_OPEN_SAMPLE;
                depositStateStartTime = getRuntime();
                break;

            case WAIT_OUTTAKE_OPEN_SAMPLE:
                if (elapsed > 0.15) {
                    linearSlides.moveSlidesToPositionInches(0);
                    if (useLowTransfer) {
                        depositAssembly.TransferSampleLow();
                    } else {
                        depositAssembly.TransferSample();
                    }
                    depositState = DepositSequenceState.DONE_TRUE_SAMPLE;
                }
                break;

            case DONE_TRUE_SAMPLE:
                depositState = DepositSequenceState.IDLE;
                break;

            // depositSampleToggle == false
            case EXTEND_SLIDES_SCORE_SAMPLE:
                linearSlides.moveSlidesToPositionInches(31);
                depositAssembly.ScoreSample();
                depositState = DepositSequenceState.DONE_FALSE_SAMPLE;
                break;

            case DONE_FALSE_SAMPLE:
                depositState = DepositSequenceState.IDLE;
                break;

            // depositSpecimenToggle == true
            case RETRACT_SLIDES_SPECIMEN_SCORE:
                linearSlides.setKP(0.005);
                linearSlides.moveSlidesToPositionInches(5);
                intakeAssembly.ExtendSlidesToPos(5);
                intakeAssembly.UnlockIntake();
                intakeAssembly.PivotClawUp();
                intakeAssembly.RotateClaw0();
                intakeAssembly.IntakeFlickerUp();
                depositState = DepositSequenceState.RETRACT_SLIDES_SPECIMEN_GRAB;
                depositStateStartTime = getRuntime();
                break;

            case RETRACT_SLIDES_SPECIMEN_GRAB:
                if (elapsed > 0.5) {
                    depositAssembly.OpenOuttakeClaw();
                    linearSlides.moveSlidesToPositionInches(2);
                    depositAssembly.GrabSpecimen();
                    depositState = DepositSequenceState.DONE_TRUE_SPECIMEN;
                }
                break;

            case DONE_TRUE_SPECIMEN:
                if (elapsed > 1.5) {
                    linearSlides.moveSlidesToPositionInches(0);
                    depositAssembly.GrabSpecimen();
                    depositState = DepositSequenceState.IDLE;
                }
                break;

            // depositSpecimenToggle == false
            case CLOSE_OUTTAKE_SPECIMEN:
                depositAssembly.CloseOuttakeClaw();
                depositState = DepositSequenceState.WAIT_OUTTAKE_CLOSE_SPECIMEN;
                depositStateStartTime = getRuntime();
                break;

            case WAIT_OUTTAKE_CLOSE_SPECIMEN:
                if (elapsed > 0.2) {
                    intakeAssembly.IntakeFlickerVertical();
                    linearSlides.moveSlidesToPositionInches(13);
                    intakeAssembly.UnlockIntake();
                    intakeAssembly.ExtendSlidesToPos(5);
                    depositState = DepositSequenceState.DONE_FALSE_SPECIMEN;
                    depositStateStartTime = getRuntime();
                }
                break;

            case DONE_FALSE_SPECIMEN:
                if (elapsed > 0.5) {
                    depositAssembly.ScoreSpecimen();
                    depositState = DepositSequenceState.IDLE;
                }
                break;
        }
    }

    // ----------------------------------------------------------------
    // ---------------- NEW: B Toggle Intake FSM ----------------------
    // ----------------------------------------------------------------

    private enum BIntakeSequenceState {
        IDLE,

        // --- B SEQUENCE 1 (bSequenceToggle == true) ---
        B_START_1,
        B_WAIT_UNLOCK_INTAKE,
        B_EXTEND_SLIDES_FULL,
        B_SET_PIVOT_MID,
        B_OPEN_CLAW,
        B_DONE_1,

        // --- B SEQUENCE 2 (bSequenceToggle == false) ---
        B_START_2,
        B_WAIT_CLOSE_CLAW,
        B_ROTATE_UP,
        B_WAIT_ROTATE_UP,
        B_EXTEND_SLIDES,
        B_WAIT_EXTEND_SLIDES,
        B_CLOSE_OUTTAKE_CLAW,
        B_WAIT_OUTTAKE_CLAW,
        // Variation: "stop and not do any transferring" after moving in
        B_DONE_2
    }

    private BIntakeSequenceState bIntakeState = BIntakeSequenceState.IDLE;
    private double bIntakeStateStartTime;

    private void startBIntakeSequence(boolean sequenceOne) {
        if (sequenceOne) {
            // B Sequence 1
            bIntakeState = BIntakeSequenceState.B_START_1;
        } else {
            // B Sequence 2
            bIntakeState = BIntakeSequenceState.B_START_2;
        }
        bIntakeStateStartTime = getRuntime();
    }

    private void updateBIntakeSequence() {
        double elapsed = getRuntime() - bIntakeStateStartTime;

        switch (bIntakeState) {
            case IDLE:
                // Do nothing until triggered
                break;

            // ------------------------------------------------
            // B Sequence 1 (do same steps as "A" Sequence 2)
            // ------------------------------------------------
            case B_START_1:
                intakeAssembly.UnlockIntake();
                bIntakeState = BIntakeSequenceState.B_WAIT_UNLOCK_INTAKE;
                bIntakeStateStartTime = getRuntime();
                break;

            case B_WAIT_UNLOCK_INTAKE:
                if (elapsed > 0.15) {
                    intakeAssembly.ExtendSlidesFull();
                    bIntakeState = BIntakeSequenceState.B_EXTEND_SLIDES_FULL;
                    bIntakeStateStartTime = getRuntime();
                }
                break;

            case B_EXTEND_SLIDES_FULL:
                // Pivot mid, flicker up, same as A's sequence 2
                intakeAssembly.PivotClawMid();
                intakeAssembly.IntakeFlickerUp();
                intakeAssembly.RotateClaw0();
                bIntakeState = BIntakeSequenceState.B_SET_PIVOT_MID;
                bIntakeStateStartTime = getRuntime();
                break;

            case B_SET_PIVOT_MID:
                if (elapsed > 0.3) {
                    intakeAssembly.OpenClaw();
                    bIntakeState = BIntakeSequenceState.B_OPEN_CLAW;
                    bIntakeStateStartTime = getRuntime();
                }
                break;

            case B_OPEN_CLAW:
                // Done with B Sequence 1
                bIntakeState = BIntakeSequenceState.B_DONE_1;
                break;

            case B_DONE_1:
                bIntakeState = BIntakeSequenceState.IDLE;
                break;

            // ------------------------------------------------
            // B Sequence 2 (similar to A's Sequence 1 but no transferring)
            // ------------------------------------------------
            case B_START_2:
                lockDpad = true;
                depositAssembly.HangAuto();
                intakeAssembly.CloseClaw();
                // You can open or close outtake as you wish. If you want it closed, skip openOuttakeClaw.
                // depositAssembly.OpenOuttakeClaw(); // (Optional)
                linearSlides.moveSlidesToPositionInches(0);
                intakeAssembly.IntakeFlickerVertical();
                bIntakeState = BIntakeSequenceState.B_WAIT_CLOSE_CLAW;
                bIntakeStateStartTime = getRuntime();
                break;

            case B_WAIT_CLOSE_CLAW:
                if (elapsed > 0.25) {
                    intakeAssembly.RotateClaw90();
                    intakeAssembly.PivotClawMid();
                    bIntakeState = BIntakeSequenceState.B_ROTATE_UP;
                    bIntakeStateStartTime = getRuntime();
                }
                break;

            case B_ROTATE_UP:
                if (elapsed > 0.1) {
                    // Move slides out some distance if you want
                    intakeAssembly.ExtendSlidesToPos(7);
                    bIntakeState = BIntakeSequenceState.B_WAIT_ROTATE_UP;
                    bIntakeStateStartTime = getRuntime();
                }
                break;

            case B_WAIT_ROTATE_UP:
                if (elapsed > 0.3) {
                    intakeAssembly.LockIntake();
                    bIntakeState = BIntakeSequenceState.B_CLOSE_OUTTAKE_CLAW;
                    bIntakeStateStartTime = getRuntime();
                }
                break;

            case B_CLOSE_OUTTAKE_CLAW:
                bIntakeState = BIntakeSequenceState.B_WAIT_OUTTAKE_CLAW;
                break;

            case B_WAIT_OUTTAKE_CLAW:
                bIntakeState = BIntakeSequenceState.B_DONE_2;
                break;

            case B_DONE_2:
                lockDpad = false;
                bIntakeState = BIntakeSequenceState.IDLE;
                break;
        }
    }
}
