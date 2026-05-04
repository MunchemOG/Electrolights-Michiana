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
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.launcher.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.launcher.PositionalHood;
import org.firstinspires.ftc.teamcode.subsystems.transfer.Transfer;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
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

@Autonomous
@Configurable
public class Red18BallSpamLinear extends NextFTCOpMode {

    public Red18BallSpamLinear() {
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE, Intake.INSTANCE, Transfer.INSTANCE, PositionalHood.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))
        );
    }

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private Paths paths;

    public Pose start = new Pose(112.158, 135.289, Math.toRadians(90));

    private boolean preloadSpinReal = false;

    private Command intakeOn = new LambdaCommand()
            .setStart(() -> Intake.INSTANCE.setPower(-1));
    private Command reverseIntakeForMe = new LambdaCommand()
            .setStart(() -> Intake.INSTANCE.setPower(0.5));
    private Command transferOn = new LambdaCommand()
            .setStart(() -> Transfer.INSTANCE.setMotorPower(-1));
    private Command openTransfer = new LambdaCommand()
            .setStart(() -> Transfer.INSTANCE.setServoPosition(0.3));
    private Command closeTransfer = new LambdaCommand()
            .setStart(() -> Transfer.INSTANCE.setServoPosition(0.635));
    private SequentialGroup shoot = new SequentialGroup(
            openTransfer, new Delay(0.05), transferOn, new Delay(0.4),
            new LambdaCommand().setStart(() -> Transfer.INSTANCE.setMotorPower(0)), closeTransfer
    );

    @Override
    public void onInit() {
        telemetry.addLine("Initializing...");
        telemetry.update();

        follower = PedroComponent.follower();

        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        limelight.start();

        paths = new Paths(follower);
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower.setStartingPose(start);
        follower.update();

        telemetry.addLine("Initialized!");
        telemetry.update();
    }

    public Command buildAuto() {
        return new SequentialGroup(
                new Delay(0.5),
                new FollowPath(paths.preloadLaunch, true, 1.0),
                shoot,
                intakeOn, transferOn, closeTransfer,
                new FollowPath(paths.intakeSet2, true, 1.0),
                new FollowPath(paths.launchSet2, true, 1.0),
                shoot,
                intakeOn, transferOn,
                new FollowPath(paths.resetAndIntake1, true, 1.0),
                new Delay(0.1),
                new FollowPath(paths.moverBacker, true, 1.0),
                new Delay(1.0),
                new FollowPath(paths.launchSpam1, true, 1.0),
                shoot,
                intakeOn, transferOn,
                new FollowPath(paths.resetAndIntake2, true, 1.0),
                new Delay(2.0),
                new FollowPath(paths.launchSpam2, true, 1.0),
                shoot,
                intakeOn, transferOn,
                new FollowPath(paths.intakeSet1, true, 1.0),
                new FollowPath(paths.launchSet1, true, 1.0),
                shoot,
                intakeOn, transferOn,
                new FollowPath(paths.intakeSet3, true, 1.0),
                new FollowPath(paths.launchSet3, true, 1.0),
                shoot,
                new FollowPath(paths.teleOpPark, true, 1.0)
        );
    }

    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        preloadSpinReal = true;
        shooter(1085);
        buildAuto().schedule();
    }

    @Override
    public void onUpdate() {
        follower.update();

        Pose currPose = follower.getPose();
        double robotHeading = currPose.getHeading();
        Vector robotToGoalVector = new Vector(
                currPose.distanceFrom(new Pose(138, 141)),
                Math.atan2(141 - currPose.getY(), 138 - currPose.getX())
        );
        Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity());
        shooter(results[0].floatValue() + 10);
        PositionalHood.hoodToPos(results[1]);
        Storage.currentPose = follower.getPose();
    }

    @Override
    public void onStop() {
        Storage.currentPose = follower.getPose();
        follower.breakFollowing();
        telemetry.addLine("Auto Stopped.");
        telemetry.update();
    }

    public class Paths {
        public PathChain preloadLaunch, intakeSet2, launchSet2, resetAndIntake1,
                moverBacker, launchSpam1, resetAndIntake2, launchSpam2,
                intakeSet1, launchSet1, intakeSet3, launchSet3, teleOpPark;

        public Paths(Follower follower) {
            preloadLaunch = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(112.158, 135.289),
                            new Pose(110.000, 110.500),
                            new Pose(100.000, 100.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(46))
                    .setVelocityConstraint(1.0).setTValueConstraint(0.8).build();

            intakeSet2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(100.000, 100.000),
                            new Pose(85, 56.798),
                            new Pose(130, 60.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(-15))
                    .setVelocityConstraint(1.0).setTValueConstraint(0.8).build();

            launchSet2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(130, 60.000),
                            new Pose(104.000, 67.000),
                            new Pose(92.000, 94.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(47))
                    .setVelocityConstraint(0.3).setTValueConstraint(0.95).build();

            resetAndIntake1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(92.000, 94.000),
                            new Pose(104.000, 67.000),
                            new Pose(131, 63)))
                    .setLinearHeadingInterpolation(Math.toRadians(47), Math.toRadians(10))
                    .setVelocityConstraint(1.0).setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeOn)
                    .addTemporalCallback(0.1, transferOn).build();

            moverBacker = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(130.5, 63.5), new Pose(130, 59.75)))
                    .setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(45))
                    .setVelocityConstraint(1.0).setTValueConstraint(0.8).build();

            launchSpam1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(130, 58.25),
                            new Pose(104.000, 67.000),
                            new Pose(92.000, 94.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(46))
                    .setVelocityConstraint(0.3).setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(124, 61), reverseIntakeForMe, 0.3).build();

            resetAndIntake2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(92.000, 94.000),
                            new Pose(104.000, 67.000),
                            new Pose(130.5, 62)))
                    .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(20))
                    .setVelocityConstraint(1.0).setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeOn)
                    .addTemporalCallback(0.1, transferOn).build();

            launchSpam2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(130.5, 63),
                            new Pose(104.000, 67.000),
                            new Pose(92, 94)))
                    .setLinearHeadingInterpolation(Math.toRadians(17), Math.toRadians(45))
                    .setVelocityConstraint(0.3).setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(118, 64), reverseIntakeForMe, 0.4).build();

            intakeSet1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(92, 94),
                            new Pose(104, 87),
                            new Pose(124.665, 86.898)))
                    .setTangentHeadingInterpolation()
                    .setVelocityConstraint(1.0).setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeOn)
                    .addTemporalCallback(0.1, transferOn).build();

            launchSet1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(124.665, 86.898), new Pose(92.000, 94.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(47))
                    .setVelocityConstraint(0.3).setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(102, 86), reverseIntakeForMe, 0.4).build();

            intakeSet3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(92.000, 94.000),
                            new Pose(88.000, 89.500),
                            new Pose(75, 28.615),
                            new Pose(129.151, 38)))
                    .setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(0))
                    .setVelocityConstraint(1.0).setTValueConstraint(0.8)
                    .addTemporalCallback(0.1, intakeOn)
                    .addTemporalCallback(0.1, transferOn).build();

            launchSet3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(129.151, 36.938), new Pose(92.000, 94.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(47))
                    .setVelocityConstraint(0.3).setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(107, 69), reverseIntakeForMe, 0.7).build();

            teleOpPark = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(92.000, 94.000), new Pose(123, 77.5)))
                    .setConstantHeadingInterpolation(Math.toRadians(90)).build();
        }
    }
}
