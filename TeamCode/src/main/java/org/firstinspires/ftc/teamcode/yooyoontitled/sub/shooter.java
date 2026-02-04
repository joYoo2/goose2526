package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;


public class shooter extends SubsystemBase{
    private final Robot robot = Robot.getInstance();

    // Servo positions
    public static final double STOPPER_OPEN = 0.75;
    public static final double STOPPER_CLOSED = 0.26;

    // Thresholds for shooting (stopper opens earlier to give it time to move)
    public static final double STOPPER_VELOCITY_THRESHOLD = 300; // Open stopper when within 300 RPM of target
    public static final double INTAKE_VELOCITY_THRESHOLD = 0.95;   // Start intake when at 95% of target velocity
    public static final double ALIGNMENT_THRESHOLD_DEG = 5.0;    // Alignment threshold in degrees

    // Distance-to-speed lookup table
    public static final InterpLUT lookUpAutoShoot = new InterpLUT();

    // LUT bounds
    public static final double MIN_DISTANCE = 3.0;
    public static final double MAX_DISTANCE = 15.0;
    public static final int MIN_SPEED = 200;
    public static final int MAX_SPEED = 1500;

    static {
        lookUpAutoShoot.add(3.0, 600);
        lookUpAutoShoot.add(4.0, 650);
        lookUpAutoShoot.add(5.0, 700);
        lookUpAutoShoot.add(6.0, 750);
        lookUpAutoShoot.add(7.0, 775);
        lookUpAutoShoot.add(8.0, 800);
        lookUpAutoShoot.add(9.0, 850);
        lookUpAutoShoot.add(10.0, 950);
        lookUpAutoShoot.add(11.0, 1000);
        //far
        lookUpAutoShoot.add(12.0, 920);
        lookUpAutoShoot.add(13.0, 1020);
        lookUpAutoShoot.add(14.0, 1250);
        lookUpAutoShoot.add(15.0, 1350);
        lookUpAutoShoot.add(10000.0, 1300);
        lookUpAutoShoot.createLUT();
    }

    /**
     * Calculates shooter speed based on distance in feet
     */
    public static int calculateShooterSpeed(double distanceFeet) {
        if (distanceFeet < MIN_DISTANCE) {
            return MIN_SPEED;
        } else if (distanceFeet > MAX_DISTANCE) {
            return MAX_SPEED;
        } else {
            return (int) lookUpAutoShoot.get(distanceFeet);
        }
    }

    public void init(){

    }

    public void shoot(int speed){
        if(robot.shooter1.getVelocity() > speed){
            robot.shooter1.setVelocity(robot.shooter1.getVelocity());
            robot.shooter2.setVelocity(robot.shooter2.getVelocity());
            shooterReady = true;
        }else{
            robot.shooter1.set(1);
            robot.shooter2.set(1);
            shooterReady = false;
        }
    }

    public void shootsetspeed(int speed){
        if(robot.shooter1.getVelocity() > speed){
            robot.shooter1.setVelocity(robot.shooter1.getVelocity());
            robot.shooter2.setVelocity(robot.shooter2.getVelocity());
        }else{
            robot.shooter1.set(1);
            robot.shooter2.set(1);
        }
    }

    /**
     * Auto-shoot method that intelligently controls stopper and intake based on shooter velocity
     * Uses the same logic as testing.java for reliable shooting
     */
    public void shootAuto(){
        int targetSpeed = 920;
        double currentVelocity = robot.shooter1.getVelocity();

        // Spin up shooters
        if(currentVelocity > targetSpeed){
            robot.shooter1.setVelocity(robot.shooter1.getVelocity());
            robot.shooter2.setVelocity(robot.shooter2.getVelocity());
        }else{
            robot.shooter2.set(1);
            robot.shooter1.set(1);
        }

        // Stage 1: Open stopper early (gives servo time to move)
        boolean stopperReady = currentVelocity > (targetSpeed - STOPPER_VELOCITY_THRESHOLD);

        // Stage 2: Start intake only when velocity is ready (using multiplier)
        boolean intakeReady = currentVelocity > (targetSpeed * INTAKE_VELOCITY_THRESHOLD);

        // Open stopper as soon as velocity is close (lower threshold)
        if(stopperReady){
            robot.stopperServo.set(STOPPER_OPEN);
            shooterReady = true;
        } else {
            robot.stopperServo.set(STOPPER_CLOSED);
            shooterReady = false;
        }

        // Start feeding when velocity is ready
        if(intakeReady){
            robot.intake.start();
        } else {
            robot.intake.stop();
        }
    }

    public void stop(){
        robot.shooter1.set(0);
        robot.shooter2.set(0);
    }

    public void reverse(){
        robot.shooter1.set(-1);
        robot.shooter2.set(-1);
    }

    //periodic runs in a loop
    @Override
    public void periodic(){
        //updateFlywheel();
    }
}