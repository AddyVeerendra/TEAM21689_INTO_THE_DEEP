package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareClasses.DepositAssembly;
import org.firstinspires.ftc.teamcode.HardwareClasses.IntakeAssemblyClaw;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "AAAAHH SIGMA TELEOP STATES HYPERDRIVE")
public class StatesTeleopHyperdrive extends LinearOpMode {

    private final double DECAY_RATE = 0.2; // Adjust for smoothness (0.03 - 0.07 works well)

    private IntakeAssemblyClaw intakeAssembly;
    private DepositAssembly depositAssembly;
    private LinearSlide linearSlides;

    private Follower follower;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

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

    public int times = 0;

    // Variables for auto-align functionality
    private boolean isAutoAligning = false; // Track if the robot is in auto-align mode
    private boolean isAlignButtonPressed = false; // Prevent multiple align triggers

    public Path alignmentPath;

    private enum AutoAlignState {
        IDLE, WAIT_FOR_TVALUE, WAIT_FOR_FIRST_ALIGN, WAIT_FOR_SLIDES, START_SECOND_ALIGN, WAIT_FOR_SECOND_ALIGN, DONE
    }

    private AutoAlignState autoAlignState = AutoAlignState.IDLE;

    // Add a new state machine for the sequence triggered by X
    private enum TeleopSequenceState {
        IDLE,
        MOVE_TO_HUMAN_PLAYER,
        WAIT_FOR_HUMAN_PLAYER,
        MOVE_TO_CHAMBER,
        WAIT_FOR_CHAMBER,
        SCORE_SPECIMEN,
        MOVE_BACK,
        RESET_CYCLE,
        DONE
    }

    private TeleopSequenceState teleopSequenceState = TeleopSequenceState.IDLE;
    private int teleopCycleCount = 0;
    private Timer teleopPathTimer = new Timer();

    private boolean isBreakingOutOfSequence = false;

    // Add these near your other variable declarations:
    private double currentY = 0.0, currentX = 0.0, currentRx = 0.0;
    private double lastUpdateTime = 0.0;

    @Override
    public void runOpMode() {
        // Initialize Drive motors

        String[] motorNames = {"slideMotorLeft", "slideMotorRight"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE};
        linearSlides = new LinearSlide(hardwareMap, motorNames, directions, 81.5625, -37.5, 37.5);

        intakeAssembly = new IntakeAssemblyClaw(hardwareMap);
        depositAssembly = new DepositAssembly(hardwareMap);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.75);

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

        follower.startTeleopDrive();
        times = 0;

        // MAIN LOOP
        while (opModeIsActive()) {

            handleAutoAlign();

            if (isAutoAligning) {
                // Check if the robot has finished aligning
                if (gamepad2.left_bumper || follower.isRobotStuck()) {
                    isAutoAligning = false; // Alignment is complete
                    follower.breakFollowing();
                    follower.startTeleopDrive(); // Resume teleop driving
                    autoAlignState = AutoAlignState.IDLE;
                }
            } else {
                if (!isAutoAligning  && teleopSequenceState == TeleopSequenceState.IDLE) {
                    double speedMultiplier = 1 - (0.7 * gamepad2.left_trigger);
                    final double DECAY_STEP = 0.02;
                    double deadband = 0.05;

// Read target joystick values
                    double targetY = -gamepad2.left_stick_y * speedMultiplier; // Forward/Back
                    double targetX = -gamepad2.left_stick_x * 1.1 * speedMultiplier; // Strafing
                    double targetRx = Range.clip(-gamepad2.right_stick_x * speedMultiplier, -0.7, 0.7); // Rotation

// For each axis, if the joystick is active, snap to target; otherwise, apply decay
// only if the current value is above 0.5.
                    if (Math.abs(targetY) > deadband) {
                        currentY = targetY;
                    } else {
                        if (Math.abs(currentY) > 0.5) {
                            if (currentY > DECAY_STEP)
                                currentY -= DECAY_STEP;
                            else if (currentY < -DECAY_STEP)
                                currentY += DECAY_STEP;
                            else
                                currentY = 0;
                        } else {
                            currentY = 0;
                        }
                    }

                    if (Math.abs(targetX) > deadband) {
                        currentX = targetX;
                    } else {
                        if (Math.abs(currentX) > 0.5) {
                            if (currentX > DECAY_STEP)
                                currentX -= DECAY_STEP;
                            else if (currentX < -DECAY_STEP)
                                currentX += DECAY_STEP;
                            else
                                currentX = 0;
                        } else {
                            currentX = 0;
                        }
                    }

                    if (Math.abs(targetRx) > deadband) {
                        currentRx = targetRx;
                    } else {
                        if (Math.abs(currentRx) > 0.5) {
                            if (currentRx > DECAY_STEP)
                                currentRx -= DECAY_STEP;
                            else if (currentRx < -DECAY_STEP)
                                currentRx += DECAY_STEP;
                            else
                                currentRx = 0;
                        } else {
                            currentRx = 0;
                        }
                    }

// Pass the updated values to your follower.
                    follower.setTeleOpMovementVectors(currentY, currentX, currentRx, true);

                    if (gamepad2.b) {
                        follower.setPose(new Pose(0, 0, Math.toRadians(0)));
                        gamepad2.rumble(200);
                    }

                    // Trigger auto-align when 'A' button is pressed
                    if (gamepad2.a && !isAlignButtonPressed) {
                        isAlignButtonPressed = true; // Prevent multiple triggers
                        initiateAutoAlignBasketStart(); // Start the auto-align process
                    } else if (!gamepad2.a) {
                        isAlignButtonPressed = false; // Reset button press state
                    }
                }
            }

            if (teleopSequenceState != TeleopSequenceState.IDLE) {
                updateTeleopSequence();
            }

            // Handle sequence when X is pressed
            if (gamepad2.x && teleopSequenceState == TeleopSequenceState.IDLE) {
                startTeleopSequence();
            }

            // Reset robot position when Y is pressed
            if (gamepad2.y) {
                follower.setPose(new Pose(41, -54, Math.toRadians(-90)));
                gamepad2.rumble(200);
            }


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

            // Toggle the outtake and intake claws
            if (gamepad1.right_bumper && !rightBumperPressed) {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    depositAssembly.CloseOuttakeClaw();
                    intakeAssembly.OpenClaw();
                } else {
                    depositAssembly.OpenOuttakeClaw();
                    intakeAssembly.CloseClaw();
                }
                rightBumperPressed = true;
            } else if (!gamepad1.right_bumper) {
                rightBumperPressed = false;
            }

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

            if (follower.isRobotStuck()) {
                follower.breakFollowing();
            }

            // Update all FSMs
            updateIntakeSequence();
            updateDepositSequence();
            updateBIntakeSequence(); // <--- NEW

            // Update hardware
            linearSlides.update();
            intakeAssembly.update();
            follower.update();
        }
    }

    private void handleAutoAlign() {
        if (autoAlignState == AutoAlignState.IDLE) {
            if (gamepad2.a && !isAlignButtonPressed) {
                isAlignButtonPressed = true;
                initiateAutoAlignBasketStart();
                autoAlignState = AutoAlignState.WAIT_FOR_FIRST_ALIGN;
            } else if (!gamepad2.a) {
                isAlignButtonPressed = false;
            }
        } else {
            switch (autoAlignState) {
                case WAIT_FOR_FIRST_ALIGN:
                    if (follower.isBusy() && follower.getCurrentTValue() > 0.1) {
                        startSampleDepositSequence(false);
                        depositSampleToggle = !depositSampleToggle;
                    }
                    if (follower.isBusy() && follower.getCurrentTValue() > 0.85) {
                        follower.setMaxPower(0.3);
                    }
                    if (!follower.isBusy()) {
                        autoAlignState = AutoAlignState.WAIT_FOR_TVALUE;
                    }
                    break;
                case WAIT_FOR_TVALUE:
                    autoAlignState = AutoAlignState.WAIT_FOR_SLIDES;
                    break;
                case WAIT_FOR_SLIDES:
                    if (!linearSlides.isSlideMotorsBusy()) {
                        initiateAutoAlignBasket();
                        autoAlignState = AutoAlignState.WAIT_FOR_SECOND_ALIGN;
                    }
                    break;
                case WAIT_FOR_SECOND_ALIGN:
                    if (!follower.isBusy()) {
                        autoAlignState = AutoAlignState.DONE;
                    }
                    break;
                case DONE:
                    autoAlignState = AutoAlignState.IDLE;
                    isAutoAligning = false;  // Add this line
                    follower.startTeleopDrive();
                    break;
                default:
                    break;
            }
        }
    }

    private void initiateAutoAlignBasketStart() {
        // Build a simple path to the alignment point (you can modify this as needed)
        alignmentPath = new Path(new BezierLine(
                new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                new Point(9, 0, Point.CARTESIAN)));
        alignmentPath.setConstantHeadingInterpolation(Math.toRadians(0));

        // Follow the path to align the robot
        follower.setMaxPower(1);
        follower.followPath(alignmentPath);

        // Switch to auto-align mode
        isAutoAligning = true;
    }

    private void initiateAutoAlignBasket() {
        // Build a simple path to the alignment point (you can modify this as needed)
        alignmentPath = new Path(new BezierLine(
                new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                new Point(0, 0, Point.CARTESIAN)));
        alignmentPath.setReversed(true);
        alignmentPath.setConstantHeadingInterpolation(Math.toRadians(0));

        // Follow the path to align the robot
        follower.setMaxPower(0.75);
        follower.followPath(alignmentPath);

        // Switch to auto-align mode
        isAutoAligning = true;
    }

    private void startTeleopSequence() {
        isBreakingOutOfSequence = false;
        teleopSequenceState = TeleopSequenceState.MOVE_TO_HUMAN_PLAYER;
        teleopCycleCount = 0;
        teleopPathTimer.resetTimer();
    }

    private void updateTeleopSequence() {
        if (gamepad2.left_bumper) {
            isAutoAligning = false;
            follower.breakFollowing();
            teleopSequenceState = TeleopSequenceState.IDLE;
            follower.startTeleopDrive(); // Resume normal control
            return;
        }
        switch (teleopSequenceState) {
            case MOVE_TO_HUMAN_PLAYER:
                isAutoAligning = true;
                follower.setMaxPower(0.85);
                Path toHumanPlayer = new Path(new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(41, -51.5, Point.CARTESIAN)));
                toHumanPlayer.setConstantHeadingInterpolation(Math.toRadians(-90));
                follower.followPath(toHumanPlayer, false);
                intakeAssembly.ExtendSlidesToPos(15);
                teleopSequenceState = TeleopSequenceState.WAIT_FOR_HUMAN_PLAYER;
                times = 0;
                break;

            case WAIT_FOR_HUMAN_PLAYER:
                if (!follower.isBusy()) {
                    if (times == 0) {
                        teleopSequenceState = TeleopSequenceState.WAIT_FOR_HUMAN_PLAYER;
                        follower.startTeleopDrive();
                        follower.setTeleOpMovementVectors(0.4, 0, 0);
                        times = 1;
                        teleopPathTimer.resetTimer();
                    }

                    if (teleopPathTimer.getElapsedTimeSeconds() > 0.45) {
                        follower.breakFollowing();
                        depositAssembly.CloseOuttakeClaw();
                    }

                    if (teleopPathTimer.getElapsedTimeSeconds() > 0.6) {
                        linearSlides.moveSlidesToPositionInches(13);
                        teleopSequenceState = TeleopSequenceState.MOVE_TO_CHAMBER;
                        teleopPathTimer.resetTimer();
                    }
                }
                break;

            case MOVE_TO_CHAMBER:
                Path toChamber = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(22, -50, Point.CARTESIAN),
                        new Point(-4 + (teleopCycleCount * 1.75), -32.5, Point.CARTESIAN)));
                toChamber.setConstantHeadingInterpolation(Math.toRadians(-90));
                follower.followPath(toChamber, false);
                teleopSequenceState = TeleopSequenceState.WAIT_FOR_CHAMBER;
                times = 0;
                break;

            case WAIT_FOR_CHAMBER:
                if (follower.isBusy() && follower.getCurrentTValue() > 0.3) {
                    depositAssembly.ScoreSpecimen();
                }

                if (!follower.isBusy()) {
                    if (times == 0) {
                        teleopSequenceState = TeleopSequenceState.WAIT_FOR_CHAMBER;
                        follower.startTeleopDrive();
                        follower.setTeleOpMovementVectors(-0.4, 0, 0);
                        times = 1;
                        teleopPathTimer.resetTimer();
                    }

                    if (teleopPathTimer.getElapsedTimeSeconds() > 0.5) {
                        follower.breakFollowing();
                        linearSlides.setKP(0.005);
                        linearSlides.moveSlidesToPositionInches(4);
                    }

                    if (teleopPathTimer.getElapsedTimeSeconds() > 0.85) {
                        depositAssembly.OpenOuttakeClaw();

                        teleopSequenceState = TeleopSequenceState.MOVE_BACK;
                        depositAssembly.GrabSpecimen();
                    }
                }
                break;

            case MOVE_BACK:
                Path toHumanPlayer2 = new Path(new BezierCurve(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(40, -35, Point.CARTESIAN),
                        new Point(40, -49.5, Point.CARTESIAN)));
                toHumanPlayer2.setConstantHeadingInterpolation(Math.toRadians(-90));
                follower.followPath(toHumanPlayer2, false);
                teleopSequenceState = TeleopSequenceState.RESET_CYCLE;
                times = 0;
                break;

            case RESET_CYCLE:
                if (follower.getCurrentTValue() > 0.3) {
                    linearSlides.moveSlidesToPositionInches(0);
                }
                if (!follower.isBusy()) {
                    if (teleopCycleCount == 7) {
                        isAutoAligning = false;
                        teleopSequenceState = TeleopSequenceState.DONE;
                        return;
                    }

                    if (times == 0) {
                        teleopSequenceState = TeleopSequenceState.RESET_CYCLE;
                        follower.startTeleopDrive();
                        follower.setTeleOpMovementVectors(0.4, 0, 0);
                        times = 1;
                        teleopPathTimer.resetTimer();
                    }

                    if (teleopPathTimer.getElapsedTimeSeconds() > 0.45) {
                        follower.breakFollowing();
                        depositAssembly.CloseOuttakeClaw();
                    }

                    if (teleopPathTimer.getElapsedTimeSeconds() > 0.6) {
                        linearSlides.moveSlidesToPositionInches(13);
                        teleopCycleCount++;
                        teleopSequenceState = TeleopSequenceState.MOVE_TO_CHAMBER;
                    }
                }
                break;

            case DONE:
                teleopSequenceState = TeleopSequenceState.IDLE;
                isAutoAligning = false;  // Add this line
                follower.startTeleopDrive();
                break;
        }
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
                    depositAssembly.TransferSample();
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
