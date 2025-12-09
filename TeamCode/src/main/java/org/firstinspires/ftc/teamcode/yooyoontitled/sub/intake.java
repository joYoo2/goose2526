package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;
public class intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public void init(){

    }

    public void start(){
        robot.intakeL.set(1);
        robot.intakeR.set(1);
        //robot.hoodServo.set(0.7);
        //robot.leftShooter.set(-0.5);
        //robot.rightShooter.set(-0.5);
    }
    public void slow(){
        robot.intakeL.set(0.1);
        robot.intakeR.set(0.1);
        //robot.hoodServo.set(0.7);
        //robot.leftShooter.set(-0.5);
        //robot.rightShooter.set(-0.5);
    }

    public void setSpeed(float n){
        robot.intakeL.set(n);
        robot.intakeR.set(n);
        //robot.hoodServo.set(0.7);
        //robot.leftShooter.set(-0.5);
        //robot.rightShooter.set(-0.5);
    }

    public void startCustom(double speed){
        robot.intakeL.set(speed);
        robot.intakeR.set(speed);
        //robot.hoodServo.set(0.7);
        robot.shooterMotorL.set(-0.5);
        robot.shooterMotorR.set(-0.5);
    }

    public void stop(){
        robot.intakeL.set(0);
        robot.intakeR.set(0);
        robot.shooterMotorL.set(0);
        robot.shooterMotorR.set(0);
    }

    public void stopExceptShooter(){
        robot.intakeL.set(0);
        robot.intakeR.set(0);
    }

    public void reverse(){
        robot.intakeL.set(-1);
        robot.intakeR.set(-1);
        //robot.leftShooter.set(-0.5);
        //robot.rightShooter.set(-0.5);
    }

    @Override
    public void periodic(){

    }
}
