package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

public class Shooter extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    private double targetSpeed = 0;

    public void setTargetSpeed(double rpm) {
        targetSpeed = rpm;
    }

    public void spinUp() {
        robot.shooterLeft.set(1);
        robot.shooterRight.set(1);
    }

    public void stop() {
        robot.shooterLeft.set(0);
        robot.shooterRight.set(0);
        targetSpeed = 0;
        Globe.shooterReady = false;
    }

    public void reverse() {
        robot.shooterLeft.set(-1);
        robot.shooterRight.set(-1);
    }

    public double calculateTargetSpeed() {
        return 800;
    }

    @Override
    public void periodic() {
        double velocity = robot.shooterLeft.getVelocity();
        Globe.shooterReady = targetSpeed > 0 && velocity > (targetSpeed * SHOOTER_INTAKE_THRESHOLD);
    }
}
