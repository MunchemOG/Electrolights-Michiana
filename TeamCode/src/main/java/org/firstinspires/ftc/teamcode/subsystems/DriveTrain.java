package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opModes.TeleOp.FarzoneTeleOpBlue.isBlueFar;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.FarzoneTeleOpRed.isRedFar;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpBlue.isBlue;
import static org.firstinspires.ftc.teamcode.opModes.TeleOp.TeleOpRed.isRed;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

import java.util.function.Supplier;

@Configurable
public class DriveTrain implements Subsystem {

    public static final DriveTrain INSTANCE = new DriveTrain();
    public DriveTrain() {}

    public static final MotorEx fL = new MotorEx("frontLeft").brakeMode();
    public static final MotorEx fR = new MotorEx("frontRight").brakeMode();
    public static final MotorEx bL = new MotorEx("backLeft").brakeMode();
    public static final MotorEx bR = new MotorEx("backRight").brakeMode();
    public static MotorEx flywheel  = new MotorEx("launchingmotor");
    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

    public int alliance;
    public boolean far;
    public double aimMultiplier = 0.575;

    private boolean autolock  = false;
    private boolean firsttime = true;
    private static boolean shooting = false;

    public Supplier<Double> yVCtx;

    double goalX = 138, goalY = 138;
    Pose startingpose = new Pose(72, 72, Math.toRadians(90));
    public Command localize;

    // PD constants for yaw autolock
    private static final double YAW_KP         = 0.09;
    private static final double YAW_KD         = 0.01;
    private static final double YAW_MAX        = 0.7;
    private static final double YAW_DEADBAND   = 0.3;
    private double lastError = 0, lastTime = 0;

    private static MotorEx intakeMotor;
    private static MotorEx transfer1;
    private static ServoEx transfer2;

    private static Servo    hoodServo1n;
    private static Servo    hoodServo2n;
    private static ServoEx  hoodServo1 = new ServoEx(() -> hoodServo1n);
    private static ServoEx  hoodServo2 = new ServoEx(() -> hoodServo2n);

    public static Command opentransfer = new LambdaCommand()
            .setStart(() -> transfer2.setPosition(0.35))
            .setIsDone(() -> true);
    public static Command closeTransfer = new LambdaCommand()
            .setStart(() -> transfer2.setPosition(0.635))
            .setIsDone(() -> true);
    private static Command transferOn  = new LambdaCommand()
            .setStart(() -> transfer1.setPower(-1.0))
            .setIsDone(() -> true);
    private static Command transferOff = new LambdaCommand()
            .setStart(() -> transfer1.setPower(0))
            .setIsDone(() -> true);
    private static Command shootFalse  = new LambdaCommand()
            .setStart(() -> shooting = false);

    private Command shooterCmd = new LambdaCommand().setStart(DriveTrain::shoot);

    // ── Helpers ──────────────────────────────────────────────────────────────

    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double visionYawCommand(double txDeg) {
        if (Math.abs(txDeg) < YAW_DEADBAND) { lastError = 0; return 0.0; }
        double now = System.currentTimeMillis() / 1000.0;
        double dt  = Math.max(now - lastTime, 0.001);
        double dErr = (txDeg != lastError) ? (txDeg - lastError) / dt : 0;
        double power = YAW_KP * txDeg + YAW_KD * dErr + 0.05 * Math.signum(txDeg);
        lastError = txDeg; lastTime = now;
        return aimMultiplier * clip(power, -YAW_MAX, YAW_MAX);
    }

    public static double hoodToPos(double pos) {
        if (Double.isNaN(pos)) return 0;
        new ParallelGroup(
                new SetPosition(hoodServo1,  pos),
                new SetPosition(hoodServo2, -pos)
        ).schedule();
        return pos;
    }

    public static void shoot() {
        if (!shooting) {
            shooting = true;
            new SequentialGroup(
                    opentransfer, new Delay(0.1),
                    transferOn,   new Delay(0.4),
                    transferOff, closeTransfer, shootFalse
            ).schedule();
        }
    }

    private void setAlliance() {
        if      (isBlue())    { alliance =  1; far = false; }
        else if (isRed())     { alliance = -1; far = false; }
        else if (isBlueFar()) { alliance =  1; far = true;  }
        else if (isRedFar())  { alliance = -1; far = true;  }
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    @Override
    public void initialize() {
        firsttime = true;
        shooting  = false;
        autolock  = false;

        setAlliance();

        if (alliance == 1) {
            startingpose = far ? new Pose(34, 9, Math.toRadians(90)) : new Pose(24, 72, Math.toRadians(90));
            localize = new LambdaCommand().setStart(() -> follower().setPose(new Pose(15, 90, Math.toRadians(90))));
        } else if (alliance == -1) {
            startingpose = far ? new Pose(110, 9, Math.toRadians(90)) : new Pose(120, 72, Math.toRadians(90));
            localize = new LambdaCommand().setStart(() -> follower().setPose(new Pose(129, 90, Math.toRadians(90))));
        }

        startingpose = Storage.currentPose;
        follower().setStartingPose(startingpose);

        hoodServo1n = ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo1");
        hoodServo2n = ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo2");

        follower().update();
    }

    @Override
    public Command getDefaultCommand() {
        follower().update();
        Pose   currPose    = follower().getPose();
        Vector toGoal      = new Vector(
                currPose.distanceFrom(new Pose(goalX, goalY)),
                Math.atan2(goalY - currPose.getY(), goalX - currPose.getX())
        );
        Double[] results         = calculateShotVectorandUpdateHeading(currPose.getHeading(), toGoal, follower().getVelocity());
        double   finalHeadingErr = results[2];
        yVCtx = () -> visionYawCommand(finalHeadingErr);

        if (autolock) {
            return new MecanumDriverControlled(fL, fR, bL, bR,
                    Gamepads.gamepad1().leftStickX(),
                    Gamepads.gamepad1().leftStickY(),
                    yVCtx);
        }
        return new MecanumDriverControlled(fL, fR, bL, bR,
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().rightStickX().map(it -> it * 0.75));
    }

    @Override
    public void periodic() {
        if (firsttime) {
            Gamepads.gamepad1().triangle()
                    .whenBecomesTrue(() -> autolock = true)
                    .whenBecomesFalse(() -> autolock = false);
            Gamepads.gamepad1().x().whenBecomesTrue(() -> localize.schedule());
            Gamepads.gamepad1().rightTrigger().greaterThan(0.3).whenBecomesTrue(shooterCmd);

            intakeMotor = new MotorEx("intake");
            transfer1   = new MotorEx("transfer");
            transfer2   = new ServoEx("transferServo1");

            new ParallelGroup(
                    new SetPosition(hoodServo1, 0),
                    new SetPosition(hoodServo2, 0)
            ).schedule();

            firsttime = false;
        }

        follower().update();

        if      (isBlue()) goalX = 6;
        else if (isRed())  goalX = 138;

        Pose   currPose = follower().getPose();
        Vector toGoal   = new Vector(
                currPose.distanceFrom(new Pose(goalX, goalY)),
                Math.atan2(goalY - currPose.getY(), goalX - currPose.getX())
        );
        Double[] results     = calculateShotVectorandUpdateHeading(currPose.getHeading(), toGoal, follower().getVelocity());
        double headingError  = results[2];
        double flywheelSpeed = results[0];
        double hoodAngle     = results[1];

        yVCtx = () -> visionYawCommand(headingError);

        if (Math.abs(headingError) > 50) {
            shooter((float)(flywheelSpeed * 0.75));
            aimMultiplier = 0.95;
        } else {
            shooter((float) flywheelSpeed);
            boolean moving = follower().getVelocity().getMagnitude() >= 8;
            if (!moving) aimMultiplier = Math.abs(headingError) < 10 ? 0.375 : 0.385;
            else         aimMultiplier = Math.abs(headingError) < 20 ? 0.5   : 0.9;
        }

        hoodToPos(hoodAngle);

        ActiveOpMode.telemetry().addData("alliance",      alliance);
        ActiveOpMode.telemetry().addData("far",           far);
        ActiveOpMode.telemetry().addData("aimMultiplier", aimMultiplier);
        ActiveOpMode.telemetry().addData("headingError",  headingError);
        ActiveOpMode.telemetry().addData("RobotX",        currPose.getX());
        ActiveOpMode.telemetry().addData("RobotY",        currPose.getY());
        ActiveOpMode.telemetry().update();
    }
}