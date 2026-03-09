package org.firstinspires.ftc.teamcode.yooyoontitled;

import com.pedropathing.geometry.Pose;

public class Globe {

    public enum OpModeType { AUTO, TELEOP }

    public enum GoalColor { RED_GOAL, BLUE_GOAL }

    // Mutable state
    public static OpModeType opModeType;
    public static GoalColor goalColor = GoalColor.RED_GOAL;
    public static Pose autoEndPose = new Pose(0, 0, 0);
    public static boolean shooterReady = false;

    // Intake motor speeds
    public static final double INTAKE_LOWER_SPEED        = 1.0;
    public static final double INTAKE_UPPER_PASSIVE_SPEED = 0.25;
    public static final double INTAKE_UPPER_FEED_SPEED   = 1.0;
    public static final double INTAKE_REVERSE_SPEED      = -1.0;

    // Intake lift servo positions (TBD — tune on robot)
    public static final double LIFT_LOWERED  = 0.17;
    public static final double LIFT_RAISED = 0.27;

    // Kick servo (CRServo — setPower: 0=stop, 1=outtake, -1=intake)
    public static final double KICK_OUTTAKE = 1.0;
    public static final double KICK_INTAKE  = -0.5;
    public static final double KICK_REVERSE  = -1;
    public static final double KICK_STOP    = 0.0;

    // Hood servo — static for now
    public static final double HOOD_STATIC_POS = 0.5;  // TODO: tune on robot

    // Shooter velocity constants
    public static final double SHOOTER_INTAKE_THRESHOLD     = 0.95; // fraction of target to set shooterReady

    // Goal positions (inches, Pedro coordinate system)
    public static final Pose RED_GOAL  = new Pose(144, 144, Math.toRadians(225));
    public static final Pose BLUE_GOAL = new Pose(0,   144, Math.toRadians(315));

    // Turret servos (CRServo — setPower: 0=stop, positive=right, negative=left)
    public static final double TURRET_SPEED        = 1;   // max power, tune on robot
    public static final double TURRET_KP           = 0.03;  // proportional gain, tune on robot
    public static final double TURRET_DEADBAND_DEG = 2.0;   // stop within this many degrees of center
    public static final double TURRET_SOFT_LIMIT   = 1.25;   // accumulated (power * seconds) before clamping

    // Limelight / April Tag
    public static final int LIMELIGHT_PIPELINE  = 1;  // pipeline index configured for AprilTags
    public static final int APRIL_TAG_BLUE_GOAL = 20;
    public static final int APRIL_TAG_RED_GOAL  = 24;
}
