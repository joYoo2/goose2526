package org.firstinspires.ftc.teamcode.yooyoontitled;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.pedropathing.geometry.Pose;

/**
 * Shared utility class for shooting calculations.
 * Aims at the average heading between the two corners of the goal.
 */
public class ShootingUtils {

    public static final double GOAL_LENGTH = 20; // inches

    // Blue goal at (0, 144): extends down (-Y) and right (+X)
    public static final Pose BLUE_CORNER_1 = new Pose(0, 144 - GOAL_LENGTH, 0);
    public static final Pose BLUE_CORNER_2 = new Pose(GOAL_LENGTH, 144, 0);

    // Red goal at (144, 144): extends down (-Y) and left (-X)
    public static final Pose RED_CORNER_1 = new Pose(144, 144 - GOAL_LENGTH, 0);
    public static final Pose RED_CORNER_2 = new Pose(144 - GOAL_LENGTH, 144, 0);

    /**
     * Calculates the target heading as the average of headings to both goal corners.
     *
     * @param robotPose Current robot position
     * @param goals Target goal color
     * @return Target heading in radians
     */
    public static double calculateTargetHeading(Pose robotPose, GoalColor goals) {
        Pose corner1, corner2;

        if (goals == GoalColor.BLUE_GOAL) {
            corner1 = BLUE_CORNER_1;
            corner2 = BLUE_CORNER_2;
        } else {
            corner1 = RED_CORNER_1;
            corner2 = RED_CORNER_2;
        }

        double heading1 = Math.atan2(corner1.getY() - robotPose.getY(), corner1.getX() - robotPose.getX());
        double heading2 = Math.atan2(corner2.getY() - robotPose.getY(), corner2.getX() - robotPose.getX());

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
        Pose target = (goals == GoalColor.BLUE_GOAL) ? BLUE_GOAL : RED_GOAL;

        double dx = (target.getX() - robotPose.getX()) / 12.0;
        double dy = (target.getY() - robotPose.getY()) / 12.0;

        return Math.sqrt(dx * dx + dy * dy);
    }
}