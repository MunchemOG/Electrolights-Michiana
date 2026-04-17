package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class ShooterConstants {
    public static Pose GOAL_POS_RED = new Pose(144, 144);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();
    public static double SCORE_HEIGHT = 30; //inches

    public static double SCORE_ANGLE = Math.toRadians(-20);

    public static double PASS_THROUGH_POINT_RADIUS = 5; //inches

    public static double RAISE_TIME = 0.15;

}
