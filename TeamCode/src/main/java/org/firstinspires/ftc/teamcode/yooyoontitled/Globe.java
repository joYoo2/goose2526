package org.firstinspires.ftc.teamcode.yooyoontitled;

import com.pedropathing.geometry.Pose;

public class Globe {
    public enum OpModeType {
        AUTO,
        TELEOP
    }

    public enum RandomizationMotif {
        GREEN_LEFT,
        GREEN_MIDDLE,
        GREEN_RIGHT
    }

    public enum GoalColor{
        BLUE_GOAL,
        RED_GOAL
    }

    public static RandomizationMotif randomizationMotif;
    public static GoalColor goals;
    public static OpModeType opModeType;

    public static boolean shot = false;
    public static Pose autoEndPose = new Pose(0, 0, Math.toRadians(0));
    public static double LAUNCHER_MAX_VELOCITY = 2500; // not used

    public static boolean shooterReady = false;

    // Goal positions on the field
    public static final Pose RED_GOAL = new Pose(144, 144, Math.toRadians(225));
    public static final Pose BLUE_GOAL = new Pose(0, 144, Math.toRadians(315));
}
