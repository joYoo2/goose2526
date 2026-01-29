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
    }
    public void slow(){
        robot.intakeL.set(0.1);
        robot.intakeR.set(0.1);
    }

    public void setSpeed(float n){
        robot.intakeL.set(n);
        robot.intakeR.set(n);
    }

    public void stop(){
        robot.intakeL.set(0);
        robot.intakeR.set(0);
    }

    public void reverse(){
        robot.intakeL.set(-1);
        robot.intakeR.set(-1);
    }



}