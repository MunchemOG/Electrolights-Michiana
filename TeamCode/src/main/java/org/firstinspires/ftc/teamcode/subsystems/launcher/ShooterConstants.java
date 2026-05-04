package org.firstinspires.ftc.teamcode.subsystems.launcher;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class ShooterConstants {
    public static Pose goalPosRed = new Pose(144, 144);
    public static Pose goalPosBlue = goalPosRed.mirror();
    public static double scoreHeight = 30; // inches

    public static double scoreAngle = Math.toRadians(-20);

    public static double passThroughPointRadius = 5; // inches

    public static double raiseTime = 0.15;
}
