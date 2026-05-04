package org.firstinspires.ftc.teamcode.subsystems.launcher;

import static java.lang.Double.isNaN;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class AutoShooterCalc implements Subsystem {

    public static boolean lowAngle = false;

    public static double requiredRpm;
    public static double requiredTps = (28 * requiredRpm) / 60;

    public static Double[] calculateShotVectorandUpdateHeading(double robotHeading, Vector robotToGoalVector, Vector robotVel) {
        double g = 32.174 * 12;
        double x = robotToGoalVector.getMagnitude() - ShooterConstants.passThroughPointRadius;
        double temp = robotToGoalVector.getMagnitude() / 39.37;
        double y = -1.2152 * temp * temp - 1.0003 * temp + 35.955;
        double a = ShooterConstants.scoreAngle;

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
        y = -1.2152 * newTemp * newTemp - 1.0003 * newTemp + 35.955;

        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));
        flywheelSpeed = flywheelSpeed / 39.37;

        double headingVelCompOffset = Math.atan(perpendicularComponent / ivr);
        double headingAngle = Math.toDegrees(robotHeading - robotToGoalVector.getTheta() + headingVelCompOffset);

        requiredRpm = 1.4286 * Math.pow(flywheelSpeed, 3) - 39.264 * Math.pow(flywheelSpeed, 2) + 863.57 * flywheelSpeed - 1373.9;
        requiredTps = (28 * requiredRpm) / 60;

        double c1 = (double) -13 / 376;
        double hoodTime = Math.toDegrees(hoodAngle) * c1 + (double) 475 / 188;

        ActiveOpMode.telemetry().addData("v0", flywheelSpeed);
        ActiveOpMode.telemetry().addData("rpm", requiredRpm);
        ActiveOpMode.telemetry().addData("angle", Math.toDegrees(hoodAngle));
        ActiveOpMode.telemetry().addData("time", hoodTime);
        ActiveOpMode.telemetry().addData("distance", robotToGoalVector.getMagnitude() / 39.37);

        Double[] returnValue = {requiredTps, hoodTime, headingAngle};
        return returnValue;
    }

    @Override
    public void initialize() {}

    @Override
    public void periodic() {}
}
