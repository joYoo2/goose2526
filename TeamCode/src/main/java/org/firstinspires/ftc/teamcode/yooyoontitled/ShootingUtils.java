package org.firstinspires.ftc.teamcode.yooyoontitled;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.pedropathing.geometry.Pose;

/**
 * Shared utility class for shooting calculations.
 * Contains backboard targeting constants and methods used by both TeleOp and Autonomous.
 */
public class ShootingUtils {
    // Backboard constants
    public static final double BACKBOARD_LENGTH = 24.0; // inches
    public static final double BACKBOARD_CENTER_OFFSET = 2; // 12 inches

    // Blue goal backboard centers (goal corner at 0, 144)
    // Y-axis wall: runs from (0, 144) to (0, 120) - center at (0, 132)
    // X-axis wall: runs from (0, 144) to (24, 144) - center at (12, 144)
    public static final Pose BLUE_Y_WALL_CENTER = new Pose(0, 141.5 - BACKBOARD_CENTER_OFFSET, 0);
    public static final Pose BLUE_X_WALL_CENTER = new Pose(BACKBOARD_CENTER_OFFSET, 141.5, 0);

    // Red goal backboard centers (goal corner at 141.5, 141.5)
    // Y-axis wall: runs from (141.5, 141.5) to (141.5, 120) - center at (141.5, 132)
    // X-axis wall: runs from (144, 144) to (120, 144) - center at (132, 144)
    public static final Pose RED_Y_WALL_CENTER = new Pose(141.5, 141.5 - BACKBOARD_CENTER_OFFSET, 0);
    public static final Pose RED_X_WALL_CENTER = new Pose(141.5 - BACKBOARD_CENTER_OFFSET, 141.5, 0);

    /**
     * Determines which backboard has more surface area facing the robot
     * using dot product of robot-to-corner vector with wall normal vectors.
     *
     * Each goal has two 24-inch backboards meeting at 90 degrees.
     * We aim at the center of whichever backboard faces the robot more directly.
     *
     * @param robotPose Current robot position
     * @param goals Target goal color
     * @return Pose of the optimal backboard center to target
     */
    public static Pose getOptimalBackboardTarget(Pose robotPose, GoalColor goals) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();

        if (goals == GoalColor.BLUE_GOAL) {
            // Blue goal at (0, 144)
            // Y-wall normal points +X, dot product = robotX
            // X-wall normal points -Y, dot product = 144 - robotY
            if (robotX > (141.5 - robotY)) {
                return BLUE_Y_WALL_CENTER; // Vertical wall at x=0
            } else {
                return BLUE_X_WALL_CENTER; // Horizontal wall at y=144
            }
        } else {
            // Red goal at (144, 144)
            // Y-wall normal points -X, dot product = 144 - robotX
            // X-wall normal points -Y, dot product = 144 - robotY
            // Simplifies to: if robotY > robotX, use Y-wall
            if (robotY > robotX) {
                return RED_Y_WALL_CENTER; // Vertical wall at x=144
            } else {
                return RED_X_WALL_CENTER; // Horizontal wall at y=144
            }
        }
    }

    /**
     * Calculates the target heading to the optimal backboard center
     * based on current alliance color and robot position.
     *
     * @param robotPose Current robot position
     * @param goals Target goal color
     * @return Target heading in radians
     */
    public static double calculateTargetHeading(Pose robotPose, GoalColor goals) {
        Pose targetBackboard = getOptimalBackboardTarget(robotPose, goals);

        double dx = targetBackboard.getX() - robotPose.getX();
        double dy = targetBackboard.getY() - robotPose.getY();

        return Math.atan2(dy, dx);
    }

    /**
     * Calculates the distance from the robot to the optimal backboard target.
     *
     * @param robotPose Current robot position
     * @param goals Target goal color
     * @return Distance in feet
     */
    public static double getDistanceToTargetFeet(Pose robotPose, GoalColor goals) {
        Pose targetBackboard = getOptimalBackboardTarget(robotPose, goals);

        double dx = (targetBackboard.getX() - robotPose.getX()) / 12.0; // Convert to feet
        double dy = (targetBackboard.getY() - robotPose.getY()) / 12.0; // Convert to feet

        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Returns a human-readable name for the currently targeted backboard.
     *
     * @param robotPose Current robot position
     * @param goals Target goal color
     * @return Human-readable string describing the target backboard
     */
    public static String getTargetBackboardName(Pose robotPose, GoalColor goals) {
        Pose target = getOptimalBackboardTarget(robotPose, goals);

        if (goals == GoalColor.BLUE_GOAL) {
            if (target == BLUE_Y_WALL_CENTER) {
                return "Blue Y-Wall (0, 132)";
            } else {
                return "Blue X-Wall (12, 141.5)";
            }
        } else {
            if (target == RED_Y_WALL_CENTER) {
                return "Red Y-Wall (141.5, 132)";
            } else {
                return "Red X-Wall (132, 141.5)";
            }
        }
    }
}
