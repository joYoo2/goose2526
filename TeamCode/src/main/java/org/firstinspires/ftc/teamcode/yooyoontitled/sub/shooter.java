package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;
import org.firstinspires.ftc.teamcode.yooyoontitled.ShootingUtils;

public class Shooter extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private static final int offset = 0;

    private static final InterpLUT SPEED_LUT = new InterpLUT();
    static {
        SPEED_LUT.add(0,0);
        SPEED_LUT.add(4.804, 1000);
        SPEED_LUT.add(5.138, 1005);
        SPEED_LUT.add(5.953, 1080);
        SPEED_LUT.add(6.301, 1075);
        SPEED_LUT.add(6.900, 1130);
        SPEED_LUT.add(7.670, 1195);
        SPEED_LUT.add(8.15, 1255);
        SPEED_LUT.add(9.38, 1310);
        SPEED_LUT.add(9.7, 1360);
        SPEED_LUT.add(10.3, 1430);
        SPEED_LUT.add(11, 1465);
        SPEED_LUT.add(11.3, 1485);
        SPEED_LUT.add(11.7, 1520);
        SPEED_LUT.add(12.15, 1560);
        SPEED_LUT.add(13, 1600);
        SPEED_LUT.add(14, 1650);
        SPEED_LUT.add(15, 1700);

        SPEED_LUT.add(10000, 2000);
        SPEED_LUT.createLUT();
    }

    private static final InterpLUT HOOD_LUT = new InterpLUT();
    static {
        // Initialize all values to 0.8 - tune later on robot
        HOOD_LUT.add(0, 0.8);
        HOOD_LUT.add(5, 0.8);
        HOOD_LUT.add(8, 0.8);
        HOOD_LUT.add(12, 0.8);
        HOOD_LUT.add(15, 0.8);
        HOOD_LUT.add(10000, 0.8);
        HOOD_LUT.createLUT();
    }

    private double targetSpeed = 0;
    private boolean isRunning  = false;
    private boolean stopperOpen = false;
    private boolean hasReachedFullSpeed = false;
    private boolean prevShootingActive = false;

    public void setTargetSpeed(double rpm) {
        targetSpeed = rpm;
    }

    public double getTargetSpeed() { return targetSpeed; }
    public double getCurrentSpeed() { return robot.shooterLeft.getVelocity(); }

    public void spinUp() {
        isRunning = true;
    }

    public void stop() {
        isRunning   = false;
        stopperOpen = false;
        hasReachedFullSpeed = false;
        robot.shooterLeft.set(0);
        robot.shooterRight.set(0);
        robot.stopperServo.set(STOPPER_CLOSED);
        targetSpeed = 0;
        Globe.shooterReady = false;
    }

    public void reverse() {
        robot.shooterLeft.set(-1);
        robot.shooterRight.set(-1);
    }

    public double calculateTargetSpeed(int adjustSpeed) {
        double distanceFeet = ShootingUtils.getDistanceToTargetFeet(robot.follower.getPose(), Globe.goalColor);
        return SPEED_LUT.get(distanceFeet) + offset + adjustSpeed;
    }

    /**
     * Calculate target speed from a specific pose (for autonomous pre-spinup).
     * @param pose The pose to calculate distance from
     * @param adjustSpeed RPM adjustment
     * @return Target shooter speed in RPM
     */
    public double calculateTargetSpeedFromPose(Pose pose, int adjustSpeed) {
        double distanceFeet = ShootingUtils.getDistanceToTargetFeet(pose, Globe.goalColor);
        return SPEED_LUT.get(distanceFeet) + offset + adjustSpeed;
    }

    /**
     * Calculate hood servo position based on distance to goal.
     * @param hoodAdjust Global offset to add to LUT value
     * @return Hood servo position (0.0 to 1.0)
     */
    public double calculateHoodAngle(double hoodAdjust) {
        double distanceFeet = ShootingUtils.getDistanceToTargetFeet(
            robot.follower.getPose(), Globe.goalColor);
        return Math.max(0.0, Math.min(1.0, HOOD_LUT.get(distanceFeet) + hoodAdjust));
    }

    /**
     * Calculate hood angle from a specific pose (for autonomous).
     * @param pose The pose to calculate distance from
     * @param hoodAdjust Global offset to add to LUT value
     * @return Hood servo position (0.0 to 1.0)
     */
    public double calculateHoodAngleFromPose(Pose pose, double hoodAdjust) {
        double distanceFeet = ShootingUtils.getDistanceToTargetFeet(pose, Globe.goalColor);
        return Math.max(0.0, Math.min(1.0, HOOD_LUT.get(distanceFeet) + hoodAdjust));
    }

    /**
     * Get shooter speed LUT value for telemetry display.
     * @param distanceFeet Distance to goal in feet
     * @return Shooter RPM from LUT (before offset/adjust)
     */
    public double getLUTValue(double distanceFeet) {
        return SPEED_LUT.get(distanceFeet);
    }

    /**
     * Get hood LUT value for telemetry display.
     * @param distanceFeet Distance to goal in feet
     * @return Hood servo position from LUT (before adjust)
     */
    public double getHoodLUTValue(double distanceFeet) {
        return HOOD_LUT.get(distanceFeet);
    }

    /**
     * Expose offset for telemetry.
     * @return Shooter speed offset in RPM
     */
    public int getOffset() {
        return offset;
    }

    @Override
    public void periodic() {
        double velocity = robot.shooterLeft.getVelocity();
        if (isRunning) {
            if (velocity >= targetSpeed) {
                robot.shooterLeft.setVelocity(velocity);
                robot.shooterRight.setVelocity(velocity);
            } else {
                robot.shooterLeft.set(1);
                robot.shooterRight.set(1);
            }
        } else {
            robot.shooterLeft.set(0);
            robot.shooterRight.set(0);
        }
        // Reset full-speed latch when shootingActive transitions false → true (new shooting session)
        if (Globe.shootingActive && !prevShootingActive) hasReachedFullSpeed = false;
        prevShootingActive = Globe.shootingActive;

        if (isRunning && velocity >= targetSpeed) hasReachedFullSpeed = true;
        Globe.shooterReady = Globe.shootingActive && targetSpeed > 0
                && (hasReachedFullSpeed ? velocity >= targetSpeed * Globe.RAPID_FIRE_THRESHOLD
                                        : velocity >= targetSpeed);

        // Stopper opens only when shooterReady; acts as ball gate during spinup and idle
        stopperOpen = Globe.shooterReady;
        robot.stopperServo.set(stopperOpen ? STOPPER_OPEN : STOPPER_CLOSED);
    }
}
