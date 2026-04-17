

package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10.55)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1, 0.1579597,0.00123647))
            .forwardZeroPowerAcceleration(-47)
            .lateralZeroPowerAcceleration(-83)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.025,
                    0,
                    0.00001,
                    0.029
            ))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0,
                    0,
                    0,
                    0
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.28,
                    0,
                    0.005,
                    0.2
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    1.5,
                    0,
                    0.1,
                    0.0005
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.01,
                    0,
                    0.000175,
                    0.6,
                    0.015
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0,
                    0.00025,
                    0.6,
                    0.01
            ))
            .centripetalScaling(0.0005);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .useBrakeModeInTeleOp(true)
            .leftFrontMotorName("frontLeft")
            .leftRearMotorName("backLeft")
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(79)
            .yVelocity(61);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.5)
            .strafePodX(5.0)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.9,
            0.8,
            0.8,
            0.5,
            50,
            3.5,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}