package org.firstinspires.ftc.teamcode.subsystems.launcher;

import static org.firstinspires.ftc.teamcode.subsystems.launcher.ShooterConstants.scoreHeight;
import static java.lang.Double.isNaN;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class ShooterCalc implements Subsystem {

    public static boolean lowAngle = false;

    public static double farZoneAngle = -0.34006585;
    public static double farZoneHeight = 10.25;

    public static double requiredRpm;
    public static double rpmLol = 3100;
    public static double rpmOffset = 90;
    public static double requiredTps = (28 * requiredRpm) / 60;

    public static Double[] calculateShotVectorandUpdateHeading(double robotHeading, Vector robotToGoalVector, Vector robotVel) {
        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - ShooterConstants.passThroughPointRadius;
        double temp = x / 39.37;
        double y = -4.5745 * temp * temp * temp + 25.978 * temp * temp - 48.395 * temp + 58.675;
        double a = ShooterConstants.scoreAngle;

        if (robotToGoalVector.getMagnitude() > 115) {
            x = robotToGoalVector.getMagnitude() + 10;
            ActiveOpMode.telemetry().addData("farzone", farZoneAngle);
            a = farZoneAngle;
            y = farZoneHeight;
        }

        double hoodAngle = MathFunctions.clamp(
                Math.atan(2 * y / x - Math.tan(a)),
                Math.toRadians(44.2),
                Math.toRadians(63)
        );
        if (isNaN(hoodAngle)) hoodAngle = Math.toRadians(63);

        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));

        double coordinateTheta = robotVel.getTheta() - robotToGoalVector.getTheta();
        double parallelComponent = -Math.cos(coordinateTheta) * robotVel.getMagnitude();
        double perpendicularComponent = Math.sin(coordinateTheta) * robotVel.getMagnitude();

        double vz = flywheelSpeed * Math.sin(hoodAngle);
        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
        double ivr = x / time + parallelComponent;
        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
        double ndr = nvr * time;

        hoodAngle = MathFunctions.clamp(
                Math.atan(vz / nvr),
                Math.toRadians(44.2),
                Math.toRadians(63)
        );
        if (isNaN(hoodAngle)) hoodAngle = Math.toRadians(63);

        double newTemp = (ndr + 5) / 39.37;
        if (y != farZoneHeight) {
            y = -4.5745 * newTemp * newTemp * newTemp + 25.978 * newTemp * newTemp - 48.395 * newTemp + 58.675;
        }

        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));
        flywheelSpeed = flywheelSpeed / 39.37;

        double headingVelCompOffset = Math.atan(perpendicularComponent / ivr);
        double headingAngle = Math.toDegrees(robotHeading - robotToGoalVector.getTheta() + headingVelCompOffset);

        requiredRpm = 1.4286 * Math.pow(flywheelSpeed, 3) - 39.264 * Math.pow(flywheelSpeed, 2) + 863.57 * flywheelSpeed - 1373.9 + rpmOffset;
        requiredTps = (28 * requiredRpm) / 60;

        double c1 = (double) -13 / 376;
        double hoodTime = Math.toDegrees(hoodAngle) * c1 + (double) 475 / 188;

        Double[] returnValue = {requiredTps, hoodTime, headingAngle};
        return returnValue;
    }

    @Override
    public void initialize() {}

    @Override
    public void periodic() {}
}
