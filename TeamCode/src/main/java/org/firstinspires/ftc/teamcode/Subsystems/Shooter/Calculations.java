package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.bylazar.configurables.annotations.Configurable;

import java.lang.Math;

import dev.nextftc.core.subsystems.Subsystem;

@Configurable
public class Calculations implements Subsystem {

    double ta;
    double distance;

    public static double v0;
    public static double numerator;
    public static double denominator;

    public static boolean lowangle = false;

    double dist;


    public static double requiredRPM;
    public static double requiredTPS = (28*requiredRPM)/60;

    public static float findTPS(double dist){
        numerator = 9.81 * Math.pow(dist, 2);
        denominator = (2 * Math.pow(Math.cos(Math.toRadians(63)), 2) * (dist * Math.tan(Math.toRadians(63)) - 0.85125));
        v0 = Math.sqrt(numerator / denominator);
        requiredRPM = 1.4286* Math.pow(v0, 3) - 39.264*Math.pow(v0, 2) + 863.57*v0-1373.9;
        //requiredRPM = 0.5 * (13.628 * v0 * v0 * v0 - 49.443 * v0 * v0 + 624.39 * v0 + 1106.4);
        requiredTPS = (28 * requiredRPM) / 60;
        return (float) requiredTPS;

    }
    public static float findTPS44(double dist){
        numerator = 9.81 * Math.pow(dist, 2);
        denominator = (2 * Math.pow(Math.cos(Math.toRadians(44.2)), 2) * (dist * Math.tan(Math.toRadians(44.2)) - 0.85125));
        v0 = Math.sqrt(numerator / denominator);
        requiredRPM = 1.4286* Math.pow(v0, 3) - 39.264*Math.pow(v0, 2) + 863.57*v0-1373.9;
        //requiredRPM = 0.5 * (13.628 * v0 * v0 * v0 - 49.443 * v0 * v0 + 624.39 * v0 + 1106.4);
        requiredTPS = (28 * requiredRPM) / 60;
        return (float) (0.975*requiredTPS);

    }



}