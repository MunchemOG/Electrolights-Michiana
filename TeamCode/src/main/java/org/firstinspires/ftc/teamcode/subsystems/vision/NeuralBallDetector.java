package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

/**
 * NeuralBallDetector
 *
 * Wraps the Limelight3A running the neural network on pipeline 0
 * (limelight_neural_detector_8bit.tflite — detects purple and green balls).
 *
 * Key methods:
 *   - getNearestBallTx()      → horizontal angle to nearest valid blob (degrees, + = right)
 *   - getNearestBallTy()      → vertical angle to nearest valid blob (degrees)
 *   - hasValidTarget()        → true if a ball is visible inside the field-centric heading window
 *   - getFieldHeadingToBall() → field-centric absolute heading to the nearest ball (degrees)
 *   - estimateBallDistance()  → rough ground-plane distance estimate from ty (inches)
 *
 * Field-centric heading filter:
 *   Only balls whose field-centric bearing is between MIN_HEADING_DEG (21°) and
 *   MAX_HEADING_DEG (270°) are considered valid.  This prevents the robot from
 *   chasing balls that are behind or in the wall zone.
 *
 * Usage:
 *   1. Call setFollower(follower) once after follower is created so the subsystem
 *      can compute field-centric headings.
 *   2. Check hasValidTarget() before acting on getNearestBallTx().
 */
public class NeuralBallDetector implements Subsystem {

    public static final NeuralBallDetector INSTANCE = new NeuralBallDetector();
    private NeuralBallDetector() {}

    // ── Pipeline ──────────────────────────────────────────────────────────────
    public static final int NEURAL_PIPELINE = 0;

    // ── Field-centric heading window for valid balls ───────────────────────────

    public static final double MIN_HEADING_DEG = 21.0;
    public static final double MAX_HEADING_DEG = 270.0;

    // ── Limelight mount geometry ───────────────────────────────────────────────
    // Tune these to match your robot's limelight mount.
    public static double LIMELIGHT_HEIGHT_INCHES = 8.0;   // height above ground
    public static double BALL_HEIGHT_INCHES       = 1.5;   // center of ball above ground
    public static double LIMELIGHT_PITCH_DEG      = 0.0;   // forward tilt of camera (+ = down)

    // ── Confidence threshold — neural network detector score ──────────────────
    public static double MIN_CONFIDENCE = 0.50;

    // ── State ─────────────────────────────────────────────────────────────────
    private Limelight3A limelight;
    private Follower follower;

    private double lastValidTx   = 0.0;
    private double lastValidTy   = 0.0;
    private double lastFieldBearing = 0.0;
    private boolean hasTarget    = false;

    // ── Follower injection ────────────────────────────────────────────────────
    /** Must be called once after the PedroComponent follower is available. */
    public void setFollower(Follower follower) {
        this.follower = follower;
    }

    // ── Public accessors ─────────────────────────────────────────────────────

    /** True if there is a neural-detected ball inside the heading window. */
    public boolean hasValidTarget() {
        return hasTarget;
    }

    /** Limelight tx of the nearest valid ball (°, + = right of crosshair). */
    public double getNearestBallTx() {
        return lastValidTx;
    }

    /** Limelight ty of the nearest valid ball (°, + = up). */
    public double getNearestBallTy() {
        return lastValidTy;
    }

    /** Field-centric absolute bearing to the nearest valid ball (°, 0 = +X axis). */
    public double getFieldHeadingToBall() {
        return lastFieldBearing;
    }

    /**
     * Rough ground-plane distance to ball based on ty and limelight geometry.
     * Returns inches; may return NaN if geometry is invalid.
     */
    public double estimateBallDistance() {
        double pitchRad = Math.toRadians(LIMELIGHT_PITCH_DEG + lastValidTy);
        if (Math.abs(pitchRad) < 1e-6) return Double.NaN;
        return (LIMELIGHT_HEIGHT_INCHES - BALL_HEIGHT_INCHES) / Math.tan(pitchRad);
    }

    // ── Limelight raw accessor ────────────────────────────────────────────────
    public Limelight3A getLimelight() { return limelight; }

    // ── Subsystem lifecycle ───────────────────────────────────────────────────

    @Override
    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(NEURAL_PIPELINE);
        limelight.start();
        hasTarget = false;
    }

    @Override
    public void periodic() {
        if (limelight == null) return;

        LLResult result = limelight.getLatestResult();
        hasTarget = false;

        if (result == null || !result.isValid()) {
            ActiveOpMode.telemetry().addLine("NeuralBallDetector: no limelight result");
            return;
        }

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        if (detections == null || detections.isEmpty()) {
            ActiveOpMode.telemetry().addLine("NeuralBallDetector: no detections");
            return;
        }

        // ── Find the nearest ball that passes the heading filter ───────────────
        double bestTx          = 0;
        double bestTy          = 0;
        double bestFieldBearing = 0;
        double bestScore       = -1;
        boolean foundValid     = false;

        double robotHeadingDeg = (follower != null)
                ? Math.toDegrees(follower.getPose().getHeading())
                : 0.0;

        for (LLResultTypes.DetectorResult det : detections) {
            if (det.getConfidence() < MIN_CONFIDENCE) continue;

            double tx = det.getTargetXDegrees();  // horizontal angle to blob
            double ty = det.getTargetYDegrees();  // vertical angle to blob

            // Field-centric bearing = robot heading + camera-relative tx
            // Limelight tx: positive = right, so field heading = robotHeading - tx
            // (depends on whether your limelight is front-mounted and robot-forward = 0°)
            double fieldBearing = robotHeadingDeg - tx;
            // Normalise to [0, 360)
            fieldBearing = ((fieldBearing % 360) + 360) % 360;

            // ── Heading window check ──────────────────────────────────────────
            boolean inWindow;
            if (MIN_HEADING_DEG <= MAX_HEADING_DEG) {
                inWindow = fieldBearing >= MIN_HEADING_DEG && fieldBearing <= MAX_HEADING_DEG;
            } else {
                // Wrap-around window (e.g. 330° – 30°)
                inWindow = fieldBearing >= MIN_HEADING_DEG || fieldBearing <= MAX_HEADING_DEG;
            }
            if (!inWindow) continue;

            // ── Pick the detection with highest confidence inside window ───────
            if (det.getConfidence() > bestScore) {
                bestScore        = det.getConfidence();
                bestTx           = tx;
                bestTy           = ty;
                bestFieldBearing = fieldBearing;
                foundValid       = true;
            }
        }

        if (foundValid) {
            hasTarget        = true;
            lastValidTx      = bestTx;
            lastValidTy      = bestTy;
            lastFieldBearing = bestFieldBearing;
        }

        // ── Telemetry ─────────────────────────────────────────────────────────
        ActiveOpMode.telemetry().addData("NN hasTarget",      hasTarget);
        ActiveOpMode.telemetry().addData("NN tx",             lastValidTx);
        ActiveOpMode.telemetry().addData("NN ty",             lastValidTy);
        ActiveOpMode.telemetry().addData("NN fieldBearing",   lastFieldBearing);
        ActiveOpMode.telemetry().addData("NN detectionCount", detections.size());
    }
}