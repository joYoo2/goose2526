package org.firstinspires.ftc.teamcode.yooyoontitled;

import com.pedropathing.geometry.Pose;

public class Globe {

    public enum OpModeType { AUTO, TELEOP }

    public enum GoalColor { RED_GOAL, BLUE_GOAL }

    // Mutable state
    public static OpModeType opModeType;
    public static GoalColor goalColor = GoalColor.RED_GOAL;
    public static Pose autoEndPose = new Pose(0, 0, 90);
    public static boolean shooterReady = false;
    public static boolean shootingActive = false;
    public static boolean shooterSpinup = false;  // full speed without feeding (for testing/warmup)
    public static boolean isIntaking   = false;

    // Intake motor speeds
    public static final double INTAKE_LOWER_SPEED        = 1.0;
    public static final double INTAKE_UPPER_PASSIVE_SPEED = 1;
    public static double INTAKE_UPPER_IDLE_SPEED    = 0.5;
    public static final double INTAKE_UPPER_FEED_SPEED   = 1.0;
    public static final double INTAKE_REVERSE_SPEED      = -1.0;

    // Intake lift servo positions (TBD — tune on robot)
    public static final double LIFT_LOWERED  = 0.17;
    public static final double LIFT_RAISED = 0.27;

    public static final double LIFT_GOAL = 0.6;



    // Stopper servo — gates ball feed (tune positions on robot)
    public static final double STOPPER_OPEN   = 0.15;
    public static final double STOPPER_CLOSED = 0.0;

    // Hood servo — static for now
    public static final double HOOD_STATIC_POS = 0.8;  // TODO: tune on robot

    // Shooter velocity constants
    public static double SHOOTER_IDLE_FRACTION = 0.75; // fraction of target speed when idle
    public static double RAPID_FIRE_THRESHOLD = 0.80; // fraction of target speed to keep shooterReady during rapid fire

    // Velocity compensation (shoot while moving)
    public static boolean VELOCITY_COMP_MODE = false; // enables continuous turret tracking + lead compensation
    public static double BALL_FLIGHT_SPEED = 10;     // estimated ball speed in ft/s (tunable)
    public static double LEAD_GAIN = 1.0;             // multiplier for lead compensation
    public static double SPEED_COMP_GAIN = 10;      // RPM adjustment per in/s of robot velocity toward goal

    // Goal positions (inches, Pedro coordinate system)
    public static final Pose RED_GOAL  = new Pose(141.5, 144, Math.toRadians(225));
    public static final Pose BLUE_GOAL = new Pose(0,   144, Math.toRadians(315));

    // Turret servos (CRServo — setPower: 0=stop, positive=right, negative=left)
    // KP/KI/KD live in Turret.java as public static fields (tunable via FTC Dashboard or TurretPIDTuning opmode)
    public static final double TURRET_SPEED        = 0.7;   // max power, tune on robot
    public static final double TURRET_DEADBAND_DEG = 3.0;   // stop within this many degrees of center
    public static double TURRET_LIMIT_DEG    = 110.0; // hard angle limit to prevent wire wrap
    public static final double TURRET_OFFSET       = 1.0;   // inches behind robot center

    // Limelight / April Tag
    public static final int LIMELIGHT_PIPELINE  = 0;  // pipeline index configured for AprilTags
    public static final int APRIL_TAG_BLUE_GOAL = 20;
    public static final int APRIL_TAG_RED_GOAL  = 24;
}
