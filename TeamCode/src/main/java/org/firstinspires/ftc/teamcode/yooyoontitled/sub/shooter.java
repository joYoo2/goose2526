package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;


public class shooter extends SubsystemBase{
    private final Robot robot = Robot.getInstance();

    // Servo positions
    private static final double STOPPER_OPEN = 0.52;
    private static final double STOPPER_CLOSED = 0;

    // Thresholds for shooting (stopper opens earlier to give it time to move)
    private static final double STOPPER_VELOCITY_THRESHOLD = 300; // Open stopper when within 300 RPM of target
    private static final double INTAKE_VELOCITY_THRESHOLD = 50;   // Start intake when within 50 RPM of target

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

    /**
     * Auto-shoot method that intelligently controls stopper and intake based on shooter velocity
     * Uses the same logic as testing.java for reliable shooting
     */
    public void shootAuto(){
        int targetSpeed = 1120;
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

        // Stage 2: Start intake only when velocity is ready
        boolean intakeReady = currentVelocity > (targetSpeed - INTAKE_VELOCITY_THRESHOLD);

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