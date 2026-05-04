package org.firstinspires.ftc.teamcode.subsystems.launcher;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.core.subsystems.Subsystem;

@Configurable
public class Calculations implements Subsystem {

    public static double v0;
    public static double numerator;
    public static double denominator;

    public static boolean lowAngle = false;

    public static double requiredRpm;
    public static double requiredTps = (28 * requiredRpm) / 60;

    public static float findTps(double dist) {
        numerator = 9.81 * Math.pow(dist, 2);
        denominator = (2 * Math.pow(Math.cos(Math.toRadians(63)), 2) * (dist * Math.tan(Math.toRadians(63)) - 0.85125));
        v0 = Math.sqrt(numerator / denominator);
        requiredRpm = 1.4286 * Math.pow(v0, 3) - 39.264 * Math.pow(v0, 2) + 863.57 * v0 - 1373.9;
        requiredTps = (28 * requiredRpm) / 60;
        return (float) requiredTps;
    }

    public static float findTps44(double dist) {
        numerator = 9.81 * Math.pow(dist, 2);
        denominator = (2 * Math.pow(Math.cos(Math.toRadians(44.2)), 2) * (dist * Math.tan(Math.toRadians(44.2)) - 0.85125));
        v0 = Math.sqrt(numerator / denominator);
        requiredRpm = 1.4286 * Math.pow(v0, 3) - 39.264 * Math.pow(v0, 2) + 863.57 * v0 - 1373.9;
        requiredTps = (28 * requiredRpm) / 60;
        return (float) (0.975 * requiredTps);
    }

    @Override
    public void initialize() {}

    @Override
    public void periodic() {}
}
