package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import static org.firstinspires.ftc.teamcode.subsystems.launcher.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.launcher.ShooterCalc.calculateShotVectorandUpdateHeading;
import static org.firstinspires.ftc.teamcode.opModes.teleOp.TeleOpBlue.isBlue;
import static org.firstinspires.ftc.teamcode.opModes.teleOp.TeleOpRed.isRed;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.launcher.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.launcher.PositionalHood;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

import java.util.function.Supplier;

@Configurable
public class DriveTrain implements Subsystem {

    public static final DriveTrain INSTANCE = new DriveTrain();
    public DriveTrain() {}

    // === Drive motors ===
    public static final MotorEx fL = new MotorEx("frontLeft").brakeMode();
    public static final MotorEx fR = new MotorEx("frontRight").brakeMode();
    public static final MotorEx bL = new MotorEx("backLeft").brakeMode();
    public static final MotorEx bR = new MotorEx("backRight").brakeMode();

    private IMUEx imu;

    // === Alliance / field state ===
    public int alliance;
    public boolean far;
    public boolean bum = false;

    // === Autolock (auto-aim to goal) ===
    private boolean autolock = false;
    public double aimMultiplier = 0.575;

    // === Slow mode ===
    private boolean slow = false;

    // === Hood angle tracking ===
    public double currentHoodState = 0;
    public double hoodAngleDriver = 0;
    public boolean decrease = false;

    // === Starting pose ===
    Pose startingPose = new Pose(72, 72, Math.toRadians(90));

    // === Goal position (set per alliance in periodic) ===
    double goalX = 138;
    double goalY = 138;
    static double localizeX;

    // === Far zone angle ===
    public static double farAngle = -0.37006585;

    // === Localize command (set per alliance in initialize) ===
    public Command localize;

    // === Yaw provider for autolock driving ===
    public Supplier<Double> yawCtx;

    // === PD autolock params ===
    private static final double YAW_KP = 0.09;
    private static final double YAW_KD = 0.01;
    private static final double YAW_MAX = 0.7;
    private static final double YAW_DEADBAND_DEG = 0.3;
    private double lastError = 0;
    private double lastTime = 0;

    // === Public toggle methods (called from TeleOp bindings) ===
    public void setAutoLockTrue()  { autolock = true; }
    public void setAutoLockFalse() { autolock = false; }
    public void setSlowTrue()      { slow = true; }
    public void setSlowFalse()     { slow = false; }

    public Command getLocalize() {
        if (isBlue()) {
            return new LambdaCommand()
                    .setStart(() -> follower().setPose(new Pose(15, 90, Math.toRadians(90))));
        } else if (isRed()) {
            return new LambdaCommand()
                    .setStart(() -> follower().setPose(new Pose(129, 90, Math.toRadians(90))));
        }
        return new LambdaCommand().setStart(() -> {});
    }

    /** Increments hood angle driver value. */
    public void hoodControl() {
        if (!decrease && hoodAngleDriver < 1) {
            hoodAngleDriver += 0.1;
        } else if (decrease && hoodAngleDriver > 0) {
            hoodAngleDriver = 1;
            decrease = true;
        } else {
            hoodAngleDriver = 0;
        }
    }

    public static void toggleFarAngle() {
        farAngle = (farAngle == -0.54006585) ? -0.04006585 : -0.54006585;
    }

    // === Drive default command ===
    @Override
    public Command getDefaultCommand() {
        Pose check = new Pose(72, 72, Math.toRadians(90));
        if (!isBlue() && !isRed()) {
            ActiveOpMode.telemetry().addLine("No direction set");
            bum = true;
        } else {
            if (isBlue()) {
                alliance = 1;
                far = false;
                if (startingPose.equals(check)) {
                    startingPose = new Pose(24, 72, Math.toRadians(90));
                    follower().setStartingPose(startingPose);
                }
            }
            if (isRed()) {
                alliance = -1;
                far = false;
                if (startingPose.equals(check)) {
                    startingPose = new Pose(120, 72, Math.toRadians(90));
                    follower().setStartingPose(startingPose);
                }
            }
        }

        follower().update();
        Pose currPose = follower().getPose();
        double robotHeading = currPose.getHeading();
        Vector robotToGoalVector = new Vector(
                currPose.distanceFrom(new Pose(goalX, goalY)),
                Math.atan2(goalY - currPose.getY(), goalX - currPose.getX())
        );
        Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower().getVelocity());

        if (autolock) {
            double headingError = results[2];
            yawCtx = () -> visionYawCommand(headingError);
            return new MecanumDriverControlled(
                    fL, fR, bL, bR,
                    Gamepads.gamepad1().leftStickX().map(it -> alliance * it),
                    Gamepads.gamepad1().leftStickY().map(it -> alliance * it),
                    yawCtx,
                    new FieldCentric(imu)
            );
        } else if (slow) {
            return new MecanumDriverControlled(
                    fL, fR, bL, bR,
                    Gamepads.gamepad1().leftStickX().map(it -> alliance * it * 0.4),
                    Gamepads.gamepad1().leftStickY().map(it -> alliance * it * 0.4),
                    Gamepads.gamepad1().rightStickX().map(it -> it * 0.4 * 0.75),
                    new FieldCentric(imu)
            );
        } else {
            return new MecanumDriverControlled(
                    fL, fR, bL, bR,
                    Gamepads.gamepad1().leftStickX().map(it -> alliance * it),
                    Gamepads.gamepad1().leftStickY().map(it -> alliance * it),
                    Gamepads.gamepad1().rightStickX().map(it -> it * 0.75),
                    new FieldCentric(imu)
            );
        }
    }

    @Override
    public void initialize() {
        autolock = false;
        follower = follower();

        if (!isBlue() && !isRed()) {
            ActiveOpMode.telemetry().addLine("No direction set");
            bum = true;
        } else {
            bum = false;
            if (isBlue()) {
                alliance = 1;
                far = false;
                startingPose = far
                        ? new Pose(34, 9, Math.toRadians(90))
                        : new Pose(24, 72, Math.toRadians(90));
                localize = new LambdaCommand()
                        .setStart(() -> follower().setPose(new Pose(15, 90, Math.toRadians(90))));
                localizeX = 136;
            }
            if (isRed()) {
                alliance = -1;
                far = false;
                startingPose = far
                        ? new Pose(110, 9, Math.toRadians(90))
                        : new Pose(120, 72, Math.toRadians(90));
                localize = new LambdaCommand()
                        .setStart(() -> follower().setPose(new Pose(129, 90, Math.toRadians(90))));
                localizeX = 8;
            }
            follower().setStartingPose(startingPose);
        }

        // Use pose from last auto if available
        startingPose = Storage.currentPose;
        follower().setStartingPose(startingPose);

        imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();

        // Zero hood on init
        PositionalHood.hoodPowerZero.schedule();

        follower().update();
    }

    @Override
    public void periodic() {
        follower().update();

        // Set goal based on alliance
        if (isBlue()) {
            goalX = 6;
            localizeX = 136;
        }
        if (isRed()) {
            goalX = 138;
            localizeX = 8;
        }

        Pose currPose = follower().getPose();
        double robotHeading = currPose.getHeading();
        Vector robotToGoalVector = new Vector(
                currPose.distanceFrom(new Pose(goalX, goalY)),
                Math.atan2(goalY - currPose.getY(), goalX - currPose.getX())
        );

        Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower().getVelocity());
        double headingError = results[2];
        yawCtx = () -> visionYawCommand(headingError);

        // Flywheel speed control for autoaim
        float flywheelSpeed = results[0].floatValue();
        if (headingError < -50 || headingError > 50) {
            shooter(flywheelSpeed * 0.75f);
            aimMultiplier = 0.95;
        } else {
            shooter(flywheelSpeed);
            aimMultiplier = (Math.abs(follower().getVelocity().getMagnitude()) < 8)
                    ? (Math.abs(headingError) < 10 ? 0.375 : 0.385)
                    : (Math.abs(headingError) < 20 ? 0.5 : 0.9);
        }

        // Position hood via PositionalHood subsystem
        PositionalHood.hoodToPos(results[1]);

        // Telemetry
        ActiveOpMode.telemetry().addData("far", far);
        ActiveOpMode.telemetry().addData("alliance", alliance);
        ActiveOpMode.telemetry().addData("aimMultiplier", aimMultiplier);
        ActiveOpMode.telemetry().addData("RobotX", currPose.getX());
        ActiveOpMode.telemetry().addData("RobotY", currPose.getY());
        ActiveOpMode.telemetry().addData("headingError", headingError);
        ActiveOpMode.telemetry().update();
    }

    // === Vision yaw PD controller for autolock ===
    private double visionYawCommand(double txDeg) {
        if (Math.abs(txDeg) < YAW_DEADBAND_DEG) {
            lastError = 0;
            return 0.0;
        }
        double currentTime = System.currentTimeMillis() / 1000.0;
        double deltaTime = currentTime - lastTime;
        if (deltaTime <= 0) deltaTime = 0.001;

        double errorDerivative = (txDeg != lastError) ? (txDeg - lastError) / deltaTime : 0;
        double power = YAW_KP * txDeg + YAW_KD * errorDerivative + 0.05 * Math.signum(txDeg);

        lastError = txDeg;
        lastTime = currentTime;
        return aimMultiplier * clip(power, -YAW_MAX, YAW_MAX);
    }

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // Needed to suppress "follower" field shadowing from static import
    private com.pedropathing.follower.Follower follower;
    private com.pedropathing.follower.Follower follower() {
        return dev.nextftc.extensions.pedro.PedroComponent.follower();
    }
}
