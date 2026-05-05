package org.firstinspires.ftc.teamcode.opModes.auto;

import static org.firstinspires.ftc.teamcode.subsystems.launcher.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.launcher.ShooterCalc.calculateShotVectorandUpdateHeading;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.launcher.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.launcher.PositionalHood;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.vision.NeuralBallDetector;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;

/**
 * RedFarNeuralAuto
 *
 * Starting pose: x=56, y=8, heading=90° (far-side Red)
 * Goal position (Red): (138, 141)
 * Launch pose: x=84, y=12, heading=90°
 *
 * Sequence overview:
 *   1. Drive to 3rd-row spike intake zone, collect balls, return to launch pose, shoot.
 *   2. Switch to neural-vision guided ball collection:
 *        a. Rotate/sweep to let NeuralBallDetector find the nearest ball in the
 *           heading window [21°, 270°] (field-centric).
 *        b. Drive toward that ball using a dynamically-built BezierLine.
 *        c. Run intake while approaching.
 *        d. After collection dwell, return to launch pose and shoot.
 *        e. Repeat for a second vision cycle.
 *   3. Park.
 *
 * Vision notes:
 *   - NeuralBallDetector.INSTANCE runs the Limelight3A on pipeline 0 (neural net).
 *   - The heading window (21°–270°) is enforced inside NeuralBallDetector.periodic().
 *   - During the vision-scan phase the robot holds position (follower idle) and
 *     slowly rotates until a valid target is found, then a fresh path is built
 *     and followed with PedroPathing.
 *   - If no ball is found after VISION_SCAN_TIMEOUT_S seconds the robot skips to park.
 */
@Autonomous(name = "RedFarNeuralAuto")
@Configurable
public class RedFarNeuralAuto extends NextFTCOpMode {

    // ── Constructor / Component registration ─────────────────────────────────
    public RedFarNeuralAuto() {
        addComponents(
                new SubsystemComponent(
                        Flywheel.INSTANCE,
                        Intake.INSTANCE,
                        Transfer.INSTANCE,
                        PositionalHood.INSTANCE,
                        NeuralBallDetector.INSTANCE
                ),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))
        );
    }

    // ── Constants ─────────────────────────────────────────────────────────────

    /** Red goal used for all shot calculations. */
    private static final Pose GOAL_POSE = new Pose(138, 141);

    /** Where the robot parks / shoots from after vision intake. */
    private static final Pose LAUNCH_POSE = new Pose(84, 12, Math.toRadians(90));

    /** How long (seconds) to wait for the neural detector to find a ball. */
    public static double VISION_SCAN_TIMEOUT_S = 3.0;

    /** How long (seconds) to run intake while driving toward a detected ball. */
    public static double VISION_INTAKE_DWELL_S = 1.8;

    /** PD gain for the rotation-to-ball phase (fraction of tx per loop). */
    public static double SCAN_YAW_KP = 0.008;

    /** How far in front of the detected ball's estimated position to target (inches). */
    public static double BALL_APPROACH_OFFSET_INCHES = 6.0;

    // ── State ─────────────────────────────────────────────────────────────────

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private Paths paths;

    private boolean autoRunning        = false;

    // Vision-guided intake state machine
    private enum VisionState { SCANNING, APPROACHING, COLLECTING, DONE }
    private VisionState visionState    = VisionState.SCANNING;
    private double visionScanStart     = 0;
    private double visionDwellStart    = 0;
    private int    visionCyclesLeft    = 2;   // run two neural-intake cycles
    private boolean visionCycleActive  = false;

    // ── Hardware wrappers (same pattern as other autos) ───────────────────────

    private Command intakeOn = new LambdaCommand()
            .setStart(() -> Intake.INSTANCE.setPower(-1))
            .setIsDone(() -> true);

    private Command intakeOff = new LambdaCommand()
            .setStart(() -> Intake.INSTANCE.setPower(0))
            .setIsDone(() -> true);

    private Command reverseIntake = new LambdaCommand()
            .setStart(() -> Intake.INSTANCE.setPower(0.5))
            .setIsDone(() -> true);

    private Command transferOn = new LambdaCommand()
            .setStart(() -> Transfer.INSTANCE.setMotorPower(-1))
            .setIsDone(() -> true);

    private Command openTransfer = new LambdaCommand()
            .setStart(() -> Transfer.INSTANCE.setServoPosition(0.3))
            .setIsDone(() -> true);

    private Command closeTransfer = new LambdaCommand()
            .setStart(() -> Transfer.INSTANCE.setServoPosition(0.635))
            .setIsDone(() -> true);

    /** Standard shoot sequence used throughout. */
    private SequentialGroup buildShoot() {
        return new SequentialGroup(
                openTransfer,
                new Delay(0.05),
                transferOn,
                new Delay(0.35),
                new LambdaCommand()
                        .setStart(() -> Transfer.INSTANCE.setMotorPower(0))
                        .setIsDone(() -> true),
                closeTransfer
        );
    }

    // ── onInit ────────────────────────────────────────────────────────────────

    @Override
    public void onInit() {
        telemetry.addLine("RedFarNeuralAuto: Initializing…");
        telemetry.update();

        follower = PedroComponent.follower();

        // Give the vision subsystem access to the follower for field-centric heading
        NeuralBallDetector.INSTANCE.setFollower(follower);

        new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();

        paths = new Paths(follower);
        pathTimer   = new Timer();
        opmodeTimer = new Timer();

        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        follower.update();

        telemetry.addLine("RedFarNeuralAuto: Ready!");
        telemetry.update();
    }

    // ── buildAuto ─────────────────────────────────────────────────────────────
    /**
     * Builds the deterministic portion of the auto (pre-loaded ball + 3rd-row spike).
     * Vision cycles are driven from onUpdate() after this command completes.
     */
    public Command buildAuto() {
        return new SequentialGroup(

                // ── Preload: drive to first launch position and shoot ──────────
                new FollowPath(paths.preloadToLaunch, true, 1.0),
                buildShoot(),

                // ── 3rd-row spike: intake along the far wall, return, shoot ───
                intakeOn, transferOn, closeTransfer,
                new FollowPath(paths.toThirdRowSpike, true, 1.0),
                new Delay(0.5),                      // dwell to collect balls
                new FollowPath(paths.spikeTolLaunch, true, 1.0),
                buildShoot(),

                // ── Signal that vision cycles should begin ────────────────────
                new LambdaCommand()
                        .setStart(() -> {
                            visionCycleActive = true;
                            visionState       = VisionState.SCANNING;
                            visionScanStart   = opmodeTimer.getElapsedTime();
                        })
                        .setIsDone(() -> true)
        );
    }

    // ── onStartButtonPressed ──────────────────────────────────────────────────

    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        autoRunning = true;

        // Pre-spin flywheel
        shooter(1085);

        buildAuto().schedule();
    }

    // ── onUpdate ─────────────────────────────────────────────────────────────
    /**
     * Runs every loop iteration.
     *
     *  1. Always update follower + shot calc.
     *  2. Drive the vision-intake state machine after the deterministic auto finishes.
     */
    @Override
    public void onUpdate() {
        follower.update();

        // ── Continuous shot calc (same as all other autos) ───────────────────
        Pose   currPose   = follower.getPose();
        Vector toGoal     = new Vector(
                currPose.distanceFrom(GOAL_POSE),
                Math.atan2(GOAL_POSE.getY() - currPose.getY(),
                        GOAL_POSE.getX() - currPose.getX())
        );
        Double[] results = calculateShotVectorandUpdateHeading(
                currPose.getHeading(), toGoal, follower.getVelocity());
        shooter(results[0].floatValue() + 30);
        PositionalHood.hoodToPos(results[1]);
        Storage.currentPose = currPose;

        // ── Vision state machine (only active after deterministic auto) ───────
        if (!visionCycleActive) return;
        if (visionCyclesLeft <= 0) {
            // All cycles done — drive to park if not already there
            if (visionCyclesLeft == 0) {
                visionCyclesLeft = -1; // prevent re-entry
                new FollowPath(paths.park, true, 1.0).schedule();
            }
            return;
        }

        switch (visionState) {

            // ── SCANNING: hold position, slowly rotate until ball found ───────
            case SCANNING: {
                double elapsed = opmodeTimer.getElapsedTime() - visionScanStart;

                if (elapsed > VISION_SCAN_TIMEOUT_S) {
                    // No ball found — skip remaining vision cycles and park
                    telemetry.addLine("Vision: timeout, parking");
                    visionCyclesLeft = 0;
                    break;
                }

                if (NeuralBallDetector.INSTANCE.hasValidTarget()) {
                    // Found a ball — transition to approach
                    visionState = VisionState.APPROACHING;
                    buildVisionApproachPath();
                } else {
                    // Slowly rotate robot clockwise to sweep field of view.
                    // Pedro holds the path follower idle while we nudge yaw manually.
                    // (No path is active here — follower.idle() keeps it from drifting.)
                    // A small rotation command is issued each loop via LambdaCommand chaining;
                    // here we simply note the rotation intent via telemetry — the actual
                    // rotation is handled by scheduling an incremental turn below.
                    scheduleScanRotation();
                }
                break;
            }

            // ── APPROACHING: following the dynamically-built path ─────────────
            case APPROACHING: {
                // Intake runs throughout approach (already scheduled via buildVisionApproachPath)
                // Transition to COLLECTING once the follower finishes the path
                if (!follower.isBusy()) {
                    visionDwellStart = opmodeTimer.getElapsedTime();
                    visionState      = VisionState.COLLECTING;
                }
                break;
            }

            // ── COLLECTING: dwell at ball location with intake running ─────────
            case COLLECTING: {
                double elapsed = opmodeTimer.getElapsedTime() - visionDwellStart;
                if (elapsed >= VISION_INTAKE_DWELL_S) {
                    // Stop intake, go shoot
                    Intake.INSTANCE.setPower(0);
                    Transfer.INSTANCE.setMotorPower(0);
                    visionState = VisionState.DONE;
                    scheduleReturnAndShoot();
                }
                break;
            }

            // ── DONE: waiting for shoot + return to complete before next cycle ─
            case DONE: {
                // The SequentialGroup scheduled in scheduleReturnAndShoot() will set
                // visionState back to SCANNING when it finishes, decrementing the counter.
                break;
            }
        }

        // Telemetry
        telemetry.addData("visionState",      visionState);
        telemetry.addData("visionCycles",     visionCyclesLeft);
        telemetry.addData("NN hasTarget",     NeuralBallDetector.INSTANCE.hasValidTarget());
        telemetry.addData("NN tx",            NeuralBallDetector.INSTANCE.getNearestBallTx());
        telemetry.addData("NN fieldBearing",  NeuralBallDetector.INSTANCE.getFieldHeadingToBall());
        telemetry.update();
    }

    // ── Vision helper methods ─────────────────────────────────────────────────

    /**
     * Schedules a small clockwise rotation to sweep the camera FOV.
     * Each call nudges the robot ~2° using Pedro's turn().
     */
    private void scheduleScanRotation() {
        double currentHeading = follower.getPose().getHeading();
        // Clockwise = subtract from heading
        double targetHeading = currentHeading - Math.toRadians(3);
        follower.turn(Math.toDegrees(targetHeading));
    }

    /**
     * Builds a BezierLine from the current robot pose toward the detected ball
     * and schedules it as a FollowPath. Intake is turned on for the approach.
     *
     * Ball world position is estimated from:
     *   - Robot's current pose
     *   - NeuralBallDetector.estimateBallDistance()  (ground-plane distance)
     *   - NeuralBallDetector.getNearestBallTx()      (horizontal angle)
     */
    private void buildVisionApproachPath() {
        Pose   robotPose    = follower.getPose();
        double robotHeading = robotPose.getHeading(); // radians, 0 = +X

        double distInches  = NeuralBallDetector.INSTANCE.estimateBallDistance();
        if (Double.isNaN(distInches) || distInches <= 0) {
            distInches = 20.0; // fallback: drive 20 inches forward toward bearing
        }
        // Add a small overshoot so the intake mouth passes over the ball
        distInches += BALL_APPROACH_OFFSET_INCHES;

        // Tx is positive-right from limelight; field bearing = robotHeading - tx_rad
        double txRad       = Math.toRadians(NeuralBallDetector.INSTANCE.getNearestBallTx());
        double bearingRad  = robotHeading - txRad;

        double targetX = robotPose.getX() + distInches * Math.cos(bearingRad);
        double targetY = robotPose.getY() + distInches * Math.sin(bearingRad);

        // Clamp to reasonable field bounds (Red far side: roughly x in [10,135], y in [0,144])
        targetX = Math.max(10, Math.min(135, targetX));
        targetY = Math.max(2,  Math.min(144, targetY));

        Pose targetPose = new Pose(targetX, targetY, Math.toRadians(90));

        PathChain approachPath = follower.pathBuilder()
                .addPath(new BezierLine(robotPose, targetPose))
                .setLinearHeadingInterpolation(robotPose.getHeading(), Math.toRadians(90))
                .setVelocityConstraint(0.6)
                .setTValueConstraint(0.9)
                .build();

        // Turn on intake for the approach
        Intake.INSTANCE.setPower(-1);
        Transfer.INSTANCE.setMotorPower(-0.5);

        new FollowPath(approachPath, true, 1.0).schedule();
    }

    /**
     * Schedules a return-to-launch + shoot sequence, then resets the state machine
     * for the next vision cycle.
     */
    private void scheduleReturnAndShoot() {
        Pose robotPose = follower.getPose();
        PathChain returnPath = follower.pathBuilder()
                .addPath(new BezierLine(robotPose, LAUNCH_POSE))
                .setLinearHeadingInterpolation(robotPose.getHeading(), Math.toRadians(90))
                .setVelocityConstraint(1.0)
                .setTValueConstraint(0.9)
                .build();

        visionCyclesLeft--;

        new SequentialGroup(
                new FollowPath(returnPath, true, 1.0),
                buildShoot(),
                // Reset for next cycle
                new LambdaCommand()
                        .setStart(() -> {
                            visionState     = VisionState.SCANNING;
                            visionScanStart = opmodeTimer.getElapsedTime();
                        })
                        .setIsDone(() -> true)
        ).schedule();
    }

    // ── onStop ────────────────────────────────────────────────────────────────

    @Override
    public void onStop() {
        Storage.currentPose = follower.getPose();
        follower.breakFollowing();
        telemetry.addLine("RedFarNeuralAuto: Stopped.");
        telemetry.update();
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Inner Paths class
    // ═════════════════════════════════════════════════════════════════════════

    public class Paths {

        /*
         * Pose reference summary (Red far side, field tile origin bottom-left):
         *
         *   Start:           x=56,  y=8,   h=90°
         *   Launch pose:     x=84,  y=12,  h=90°  (used after every shot)
         *   3rd-row spike:   x=128, y=38,  h=0°   (mirrors Red21 intakeSet3 endpoint ~(129,38))
         *   Red goal:        (138, 141)
         *   Park:            x=92,  y=24,  h=90°
         *
         * All coordinates match the field-coordinate system used by the other Red autos.
         * The "3rd-row spike" location is taken from Red21BallSpamLinear.intakeSet3 endpoint
         * (129.151, 38) which is the far wall ball cluster on the Red side.
         */

        public PathChain preloadToLaunch;
        public PathChain toThirdRowSpike;
        public PathChain spikeTolLaunch;
        public PathChain park;

        public Paths(Follower follower) {

            // ── Preload → first launch position ──────────────────────────────
            // Drive from start (56,8,90°) curving up to launch pose (84,12,90°).
            preloadToLaunch = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56,  8),
                            new Pose(68, 10),
                            new Pose(84, 12)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .setVelocityConstraint(0.8)
                    .setTValueConstraint(0.85)
                    .build();

            // ── Launch pose → 3rd-row spike ball cluster ──────────────────────
            // Mirrors the Red intakeSet3 approach: sweeps down-field along the far wall.
            // Intake is enabled via temporal callbacks so balls are collected en route.
            toThirdRowSpike = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(84,  12),
                            new Pose(88,  25),
                            new Pose(100, 30),
                            new Pose(129, 38)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .setVelocityConstraint(0.9)
                    .setTValueConstraint(0.85)
                    .addTemporalCallback(0.05, intakeOn)
                    .addTemporalCallback(0.05, transferOn)
                    // Reverse intake briefly near the end to settle balls
                    .addPoseCallback(new Pose(115, 35), reverseIntake, 0.3)
                    .build();

            // ── 3rd-row spike → launch pose ───────────────────────────────────
            // Return along a gentle curve to the launch position.
            spikeTolLaunch = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(129, 38),
                            new Pose(110, 25),
                            new Pose(84,  12)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .setVelocityConstraint(0.8)
                    .setTValueConstraint(0.95)
                    .build();

            // ── Final park ────────────────────────────────────────────────────
            // After all vision cycles, return near the launch pose area.
            park = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(84, 12),
                            new Pose(92, 24)))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }
    }
}