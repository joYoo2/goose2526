package org.firstinspires.ftc.teamcode.yooyoontitled;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.pedropathing.geometry.Pose;

/**
 * Shared utility class for shooting calculations.
 * Aims at the average heading between the two corners of the goal.
 */
public class ShootingUtils {

    public static final double GOAL_LENGTH_X = 20; // inches
    public static final double GOAL_LENGTH_Y = 20; // inches

    // Blue goal at (0, 144): extends down (-Y) and right (+X)
    public static final Pose BLUE_CORNER_1 = new Pose(0, 144 - GOAL_LENGTH_Y, 0);
    public static final Pose BLUE_CORNER_2 = new Pose(GOAL_LENGTH_X, 144, 0);

    // Red goal at (144, 144): extends down (-Y) and left (-X)
    public static final Pose RED_CORNER_1 = new Pose(144, 144 - GOAL_LENGTH_Y, 0);
    public static final Pose RED_CORNER_2 = new Pose(144 - GOAL_LENGTH_X, 144, 0);

    /**
     * Calculates the actual turret position offset from robot center.
     * Turret is mounted 5 inches behind robot center.
     *
     * @param robotPose Current robot center pose
     * @return Pose of the turret (offset backward by TURRET_OFFSET)
     */
    public static Pose getTurretPose(Pose robotPose) {
        double heading = robotPose.getHeading();
        // Offset backward (opposite direction of heading)
        double offsetX = -Globe.TURRET_OFFSET * Math.cos(heading);
        double offsetY = -Globe.TURRET_OFFSET * Math.sin(heading);
        
        return new Pose(
            robotPose.getX() + offsetX,
            robotPose.getY() + offsetY,
            heading  // heading stays the same
        );
    }

    /**
     * Calculates the target heading as the average of headings to both goal corners.
     *
     * @param robotPose Current robot position
     * @param goals Target goal color
     * @return Target heading in radians
     */
    public static double calculateTargetHeading(Pose robotPose, GoalColor goals) {
        // Calculate actual turret position (offset behind robot center)
        Pose turretPose = getTurretPose(robotPose);
        
        Pose corner1, corner2;

        if (goals == GoalColor.BLUE_GOAL) {
            corner1 = BLUE_CORNER_1;
            corner2 = BLUE_CORNER_2;
        } else {
            corner1 = RED_CORNER_1;
            corner2 = RED_CORNER_2;
        }

        double heading1 = Math.atan2(corner1.getY() - turretPose.getY(), corner1.getX() - turretPose.getX());
        double heading2 = Math.atan2(corner2.getY() - turretPose.getY(), corner2.getX() - turretPose.getX());

        // Average the two headings
        return (heading1 + heading2) / 2.0;
    }

    /**
     * Calculates the distance from the robot to the goal corner.
     *
     * @param robotPose Current robot position
     * @param goals Target goal color
     * @return Distance in feet
     */
    public static double getDistanceToTargetFeet(Pose robotPose, GoalColor goals) {
        // Calculate actual turret position (offset behind robot center)
        Pose turretPose = getTurretPose(robotPose);
        Pose target = (goals == GoalColor.BLUE_GOAL) ? BLUE_GOAL : RED_GOAL;

        double dx = (target.getX() - turretPose.getX()) / 12.0;
        double dy = (target.getY() - turretPose.getY()) / 12.0;

        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Calculates lead-compensated heading accounting for robot velocity.
     * Aims ahead of the goal to compensate for robot movement during ball flight.
     *
     * @param robotPose Current robot position
     * @param robotVelX Robot X velocity in inches/second
     * @param robotVelY Robot Y velocity in inches/second
     * @param goals Target goal color
     * @return Lead-compensated heading in radians
     */
    public static double calculateLeadHeading(Pose robotPose, double robotVelX, double robotVelY, GoalColor goals) {
        // Calculate actual turret position (offset behind robot center)
        Pose turretPose = getTurretPose(robotPose);
        
        // Get distance and estimate flight time
        double distanceFeet = getDistanceToTargetFeet(robotPose, goals);  // uses turret pose internally
        double flightTime = distanceFeet / Globe.BALL_FLIGHT_SPEED;

        // Calculate lead offset - where robot will have moved by time ball arrives
        // We aim at "virtual goal" position offset by robot movement
        double leadX = robotVelX * flightTime * Globe.LEAD_GAIN;
        double leadY = robotVelY * flightTime * Globe.LEAD_GAIN;

        // Get goal center position
        Pose goal = (goals == GoalColor.BLUE_GOAL) ? BLUE_GOAL : RED_GOAL;

        // Aim at goal position minus lead offset (compensates for robot movement)
        double targetX = goal.getX() - leadX;
        double targetY = goal.getY() - leadY;

        return Math.atan2(targetY - turretPose.getY(), targetX - turretPose.getX());
    }

    /**
     * Calculates shooter RPM compensation based on robot velocity toward/away from goal.
     * Moving toward goal = ball has extra momentum = less RPM needed
     * Moving away from goal = ball loses momentum = more RPM needed
     *
     * @param robotPose Current robot position
     * @param robotVelX Robot X velocity in inches/second
     * @param robotVelY Robot Y velocity in inches/second
     * @param goals Target goal color
     * @return RPM adjustment (negative if moving toward goal, positive if away)
     */
    public static double calculateSpeedCompensation(Pose robotPose, double robotVelX, double robotVelY, GoalColor goals) {
        // Calculate actual turret position (offset behind robot center)
        Pose turretPose = getTurretPose(robotPose);
        Pose goal = (goals == GoalColor.BLUE_GOAL) ? BLUE_GOAL : RED_GOAL;

        // Direction vector to goal
        double dx = goal.getX() - turretPose.getX();
        double dy = goal.getY() - turretPose.getY();
        double dist = Math.sqrt(dx * dx + dy * dy);

        if (dist < 1.0) return 0;  // too close, avoid division issues

        // Unit vector toward goal
        double ux = dx / dist;
        double uy = dy / dist;

        // Robot velocity component toward goal (dot product)
        double velTowardGoal = robotVelX * ux + robotVelY * uy;  // in/s

        // Moving toward goal = negative adjustment (need less RPM)
        // Moving away = positive adjustment (need more RPM)
        return -velTowardGoal * Globe.SPEED_COMP_GAIN;
    }
}