package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;
import org.firstinspires.ftc.teamcode.yooyoontitled.ShootingUtils;

public class Shooter extends SubsystemBase {
    private final Robot robot = Robot.getInstance();
    private static final int offset = -50;

    private static final InterpLUT SPEED_LUT = new InterpLUT();
    static {
        SPEED_LUT.add(0,0);
        SPEED_LUT.add(4.804, 1110);
        SPEED_LUT.add(5.138, 1130);
        SPEED_LUT.add(5.953, 1180);
        SPEED_LUT.add(6.301, 1195);
        SPEED_LUT.add(6.900, 1210);
        SPEED_LUT.add(7.670, 1270);
        SPEED_LUT.add(8.15, 1310);
        SPEED_LUT.add(8.85, 1330);
        SPEED_LUT.add(9.38, 1340);
        SPEED_LUT.add(9.7, 1360);
        SPEED_LUT.add(10.3, 1480);
        SPEED_LUT.add(11, 1530);
        SPEED_LUT.add(11.3, 1560);
        SPEED_LUT.add(11.7, 1610);
        SPEED_LUT.add(12.15, 1660);
        SPEED_LUT.add(13, 1700);
        SPEED_LUT.add(14, 1750);
        SPEED_LUT.add(15, 1800);

        SPEED_LUT.add(10000, 2000);
        SPEED_LUT.createLUT();
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
