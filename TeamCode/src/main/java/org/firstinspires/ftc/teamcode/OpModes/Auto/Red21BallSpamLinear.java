package org.firstinspires.ftc.teamcode.OpModes.Auto;

import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.Subsystems.Shooter.ShooterCalc.calculateShotVectorandUpdateHeading;

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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.DistanceRed;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Storage;
//import org.firstinspires.ftc.teamcode.subsystems.Storage;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;


@Autonomous
@Configurable
public class Red21BallSpamLinear extends NextFTCOpMode {
    public Red21BallSpamLinear(){
        addComponents(
                new SubsystemComponent(Flywheel.INSTANCE,DistanceRed.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))


        );
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;


    private Paths paths;
    public MotorEx intakeMotor;

    int tagId = 0;

    public Pose start = new Pose(112.158,135.289, Math.toRadians(90));


    private MotorEx transfer1;

    private ServoEx transfer2;

    public static MotorEx flywheel = new MotorEx("launchingmotor");

    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

           /* private CRServo hoodServo1n;
            private CRServo hoodServo2n;

            private CRServoEx hoodServo1 = new CRServoEx(() -> hoodServo1n);
            private CRServoEx hoodServo2 = new CRServoEx(() -> hoodServo2n);






            ParallelGroup HoodRunUp=new ParallelGroup(
                    new SetPower(hoodServo1,-1),
                    new SetPower(hoodServo2,1)
            );

            public ParallelGroup HoodPowerZero=new ParallelGroup(
                    new SetPower(hoodServo1,0),
                    new SetPower(hoodServo2,0)
            );

            public SequentialGroup HoodUp=new SequentialGroup(
                    HoodRunUp,
                    new Delay(0.18),
                    HoodPowerZero
            );

            ParallelGroup HoodRunDown=new ParallelGroup(
                    new SetPower(hoodServo1,1),
                    new SetPower(hoodServo2,-1)
            );

            public SequentialGroup HoodDown=new SequentialGroup(
                    HoodRunDown,
                    new Delay(0.17),
                    HoodPowerZero
            );

            public SequentialGroup HoodUpAuto=new SequentialGroup(
                    HoodRunUp,
                    new Delay(0.12),
                    HoodPowerZero
            );*/

    private Boolean preloadspin;

    private double preloadtps;

    private double shoottps;
    Command findTPSShoot = new LambdaCommand()
            .setStart(()->shoottps = findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
    Command findTPSPreload = new LambdaCommand()
            .setStart(()->preloadtps = findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));


    private static Servo hoodServo1n;
    private static Servo hoodServo2n;

    private static ServoEx hoodServo1 = new ServoEx(() -> hoodServo1n);
    private static ServoEx hoodServo2 = new ServoEx(() -> hoodServo2n);
    public void onInit() {
        telemetry.addLine("Initializing Follower...");

        telemetry.update();
        follower = PedroComponent.follower();
                /*Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
                limelight.pipelineSwitch(APRILTAG_PIPELINE);
                limelight.start();*/


        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(7);
        limelight.start();
        paths = new Paths(follower);
        intakeMotor = new MotorEx("intake");
        transfer1 = new MotorEx("transfer");
        transfer2 = new ServoEx("transferServo1");
        //hoodServo1n= ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo1");
        // hoodServo2n=  ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo2");
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower.setStartingPose(start);


        pathState = 0;
        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();
        hoodServo1n= ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo1");
        hoodServo2n=  ActiveOpMode.hardwareMap().get(Servo.class, "hoodServo2");

        follower.update();


    }

    public static double hoodToPos(double runtime) {
        if(Double.isNaN(runtime)!=true) {
            ActiveOpMode.telemetry().addData("runtime", runtime);
            ParallelGroup HoodRunUp = new ParallelGroup(
                    new SetPosition(hoodServo1, runtime),
                    new SetPosition(hoodServo2, -1*runtime)
            );
            HoodRunUp.schedule();
            return runtime;
        }
        else {
            ActiveOpMode.telemetry().addLine("NaN");
            return 0;
        }
    }

    private Command intakeMotorOn = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(-1));

    Command intakeMotorOff = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0));

    double distance;

    Command transferOn = new LambdaCommand()
            .setStart(()-> transfer1.setPower(-1));
    Command transferOff = new LambdaCommand()
            .setStart(() -> transfer1.setPower(0));
    Command transferOnForIntake = new LambdaCommand()
            .setStart(()-> transfer1.setPower(-1));

    public Command opentransfer = new LambdaCommand()
            .setStart(()-> {
                //`5transfer2.setPosition(-0.25);
                transfer2.setPosition(0.3);
            });

    public Command closeTransfer = new LambdaCommand()
            .setStart(() -> {
                transfer2.setPosition(0.635);
            });
    //niggers are black = (NO SHIT SHERLOCK);
    public SequentialGroup shoot = new SequentialGroup(opentransfer, new Delay(0.05), transferOn, new Delay(0.3), transferOff, closeTransfer);

    public boolean spinup = true;
    public Command spinupfalse = new LambdaCommand()
            .setStart(()-> {
                spinup=false;
            });

    Command spinupPLEASEEIsagiINEEDTHIS = new LambdaCommand()
            .setStart(()->{
                flywheel.setPower(1);
                flywheel2.setPower(-1);

            });
    public Command reverseIntakeForMe = new LambdaCommand()
            .setStart(() -> intakeMotor.setPower(0.5));
    public Command aimAtGoal = new LambdaCommand()
            .setStart(() -> {
                Pose currPose = follower.getPose();

                // 1. Calculate the vector to the goal (138, 141)
                Vector robotToGoalVector = new Vector(
                        follower.getPose().distanceFrom(new Pose(138, 141)),
                        Math.atan2(141 - currPose.getY(), 138 - currPose.getX())
                );

                // 2. Get the required heading and flywheel/hood specs from your utility
                Double[] results = calculateShotVectorandUpdateHeading(
                        currPose.getHeading(),
                        robotToGoalVector,
                        follower.getVelocity()
                );

                double targetHeading = results[2]; // Assuming index 2 is the target heading from your calc
                double flywheelSpeed = results[0];
                double hoodAngle = results[1];

                // 3. Update Hardware
                shooter((float) (flywheelSpeed + 30));
                hoodToPos(hoodAngle);

                // 4. Force Pedro Pathing to face the goal
                // This overrides the path's heading interpolation with the live target
                follower.turn(targetHeading);
            });
    public boolean lift;
    boolean lowerangle = false;
    private Boolean preloadspinreal = false;

    Command preloadSpun = new LambdaCommand().setStart(() -> preloadspinreal = true);
    Command preloadSpunReal = new LambdaCommand().setStart(() -> preloadspinreal = false);
    public boolean intake3 = false;
    Command shootIntake3 = new LambdaCommand()
            .setStart(()->intake3=true);

    public boolean intakeSpam = false;

    Command shootIntakeSpam = new LambdaCommand()
            .setStart(()->intakeSpam=true);




    public Command Auto(){
        return new SequentialGroup(

                        /*spinupPLEASEEIsagiINEEDTHIS,
                        preloadSpun*/
                new Delay(0.3),
                new FollowPath(paths.preloadLaunch,true,1.0),
                shoot,
                intakeMotorOn,
                transferOn,
                closeTransfer,

                new FollowPath(paths.intakeSet2,true,1.0),



                new FollowPath(paths.launchSet2,true,1.0),
                shoot,
                intakeMotorOn,
                transferOn,

                new FollowPath(paths.resetAndIntake1,true,1.0),
                new Delay(0.1),
                new FollowPath(paths.moverBacker,true,1.0),
                new Delay(0.3),
                //reverseIntakeForMe,
                new FollowPath(paths.launchSpam1, true, 1.0),
                shoot,

                intakeMotorOn,
                transferOn,


                new FollowPath(paths.resetAndIntake2, true, 1.0),
                new Delay(1.3),
                //reverseIntakeForMe,

                new FollowPath(paths.launchSpam2, true, 1.0),
                shoot,
                intakeMotorOn,
                transferOn,
                new FollowPath(paths.resetAndIntake2, true, 1.0),
                new Delay(1.6),
                //reverseIntakeForMe,

                new FollowPath(paths.launchSpam2, true, 1.0),
                shoot,

                intakeMotorOn,
                transferOn,
                new FollowPath(paths.intakeSet1,true,1.0),
                new FollowPath(paths.launchSet1,true,1.0),
                shoot,

                intakeMotorOn,
                transferOn,
                new FollowPath(paths.intakeSet3,true,1.0),
                new FollowPath(paths.launchSet3,true,1.0),


                new FollowPath(paths.teleOpPark,true,1.0)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
                /*flywheel.setPower(1);
                flywheel2.setPower(-1);*/

        preloadspinreal = true;
        shooter(1085);

        //int tag=MotifScanning.INSTANCE.findMotif();
        Auto().schedule();



    }

    @Override
    public void onUpdate(){
        follower.update();

                /*if(preloadspinreal) {
                    shooter(1080);
                }
                else{
                    if (DistanceRed.INSTANCE.getDistanceFromTag() != 0) {
                        shooter(findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
                        ActiveOpMode.telemetry().addData("Limelight!", findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
                    } else if (DistanceRed.INSTANCE.getDistanceFromTag() == 0 && !intake3&&!intakeSpam) {
                        shooter(1065);
                    }
                    else if(intake3){
                        shooter(1070);
                    }
                    else if(intakeSpam){
                        shooter(1065);
                    }
                }*/

        Pose currPose = follower.getPose();
        double robotHeading = follower.getPose().getHeading();
        Vector robotToGoalVector = new Vector(follower.getPose().distanceFrom(new Pose(138, 141)), Math.atan2(141 - currPose.getY(), 138 - currPose.getX()));
        //Vector v = new Vector(new Pose(138, 138));
        Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity());
        double flywheelSpeed = results[0];
        shooter((float) (flywheelSpeed+30));
        double hoodAngle = results[1];
        hoodToPos(hoodAngle);
        Storage.currentPose = follower.getPose();

    }




    @Override
    public void onStop() {
        Storage.currentPose = follower.getPose();
        follower.breakFollowing();
        telemetry.addLine("Autonomous Stopped.");
        telemetry.update();
    }




    public class Paths {
        public PathChain preloadLaunch;
        public PathChain intakeSet2;
        public PathChain launchSet2;
        public PathChain resetAndIntake1;
        public PathChain moverBacker;
        public PathChain launchSpam1;
        public PathChain resetAndIntake2;
        public PathChain launchSpam2;
        public PathChain intakeSet1;
        public PathChain launchSet1;
        public PathChain intakeSet3;
        public PathChain launchSet3;
        public PathChain teleOpPark;

        public Paths(Follower follower) {
            preloadLaunch = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(112.158,135.289),
                                    new Pose(110.000, 110.500),
                                    new Pose(100.000, 100.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90),Math.toRadians(46))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)

                    .build();

            intakeSet2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(100.000, 100.000),
                                    new Pose(85, 56.798),
                                    new Pose(130, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(-15))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)



                    .build();

            launchSet2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130, 60.000),
                                    new Pose(104.000, 67.000),
                                    new Pose(92.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(47))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    //.addTemporalCallback(0.7,reverseIntakeForMe)
                    //.addTemporalCallback(0.2,transferOff)

                    .build();

            resetAndIntake1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92.000, 94.000),
                                    new Pose(104.000, 67.000),
                                    new Pose(128, 64)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(47), Math.toRadians(20))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1,intakeMotorOn)
                    .addTemporalCallback(0.1,transferOn)

                    .build();

            moverBacker = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128, 64),
                                    new Pose(130, 59.75)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(45))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)

                    .build();

            launchSpam1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130, 58.25),
                                    new Pose(104.000, 67.000),
                                    new Pose(92.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(46))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    //.addTemporalCallback(0.9,reverseIntakeForMe)
                    .addPoseCallback(new Pose(124,61),reverseIntakeForMe,0.3)

                    .build();

            resetAndIntake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92.000, 94.000),
                                    new Pose(104.000, 67.000),
                                    new Pose(130.5, 62)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(23))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1,intakeMotorOn)
                    .addTemporalCallback(0.1,transferOn)

                    .build();

            launchSpam2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130.5, 63),
                                    new Pose(104.000, 67.000),
                                    new Pose(92, 94)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(17), Math.toRadians(47))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(118,64),reverseIntakeForMe,0.4)


                    //.addTemporalCallback(0.9,reverseIntakeForMe)
                    // .addTemporalCallback(0.7,transferOff)


                    .build();

            intakeSet1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92, 94),
                                    new Pose(104,87),

                                    new Pose(126.665, 86.898)
                            )
                    ).setTangentHeadingInterpolation()
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1,intakeMotorOn)
                    .addTemporalCallback(0.1,transferOn)


                    .build();

            launchSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.665, 86.898),

                                    new Pose(92.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(47))
                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.95)
                    .addPoseCallback(new Pose(102,86),reverseIntakeForMe,0.4)

                    //.addTemporalCallback(0.7,reverseIntakeForMe)
                    //.addTemporalCallback(0.7,transferOff)

                    .build();

            intakeSet3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(92.000, 94.000),
                                    new Pose(88.000, 89.500),
                                    new Pose(75, 28.615),
                                    new Pose(129.151, 38)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(0))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8)
                    .addTemporalCallback(0.1,intakeMotorOn)
                    .addTemporalCallback(0.1,transferOn)

                    .build();

            launchSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.151, 36.938),

                                    new Pose(84, 103)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(27))

                    .setVelocityConstraint(0.3)
                    .setTValueConstraint(0.8)
                    .addPoseCallback(new Pose(107,69),reverseIntakeForMe,0.7)
                    .addPoseCallback(new Pose(94,88),shoot,0.8)

                    //.addTemporalCallback(0.7,reverseIntakeForMe)
                    //.addTemporalCallback(0.7,transferOff)

                    .build();

            teleOpPark = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84, 103),

                                    new Pose(91, 115)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(30),Math.toRadians(90))

                    .build();
        }
    }
}