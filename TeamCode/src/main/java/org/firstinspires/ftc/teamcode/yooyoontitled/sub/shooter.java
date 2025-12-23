package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;


public class shooter extends SubsystemBase{
    private final Robot robot = Robot.getInstance();
    //private final PIDFController flywheelController = new PIDFController(FLYWHEEL_PIDF_COEFFICIENTS);



    public void init(){

    }

    public void shoot(int speed){
        //
        if(robot.shooter1.getVelocity() > speed){
            robot.shooter1.setVelocity(robot.shooter1.getVelocity());
            robot.shooter2.setVelocity(robot.shooter2.getVelocity());
            shooterReady = true;
        }else{
//            robot.leftShooter.setVelocity(1300);
//            robot.rightShooter.setVelocity(1300);
            robot.shooter1.set(1);
            robot.shooter2.set(1);
            shooterReady = false;
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
