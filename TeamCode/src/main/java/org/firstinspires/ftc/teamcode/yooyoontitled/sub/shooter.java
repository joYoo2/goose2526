package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;


public class shooter extends SubsystemBase{
    private final Robot robot = Robot.getInstance();
    //private final PIDFController flywheelController = new PIDFController(FLYWHEEL_PIDF_COEFFICIENTS);

    private double targetHoodAngle = 90-15;//MIN_HOOD_ANGLE;
    private double targetFlywheelVelocity = 0.0;

    private final InterpLUT launcherVel = new InterpLUT(); //converts from m/s to ticks/s ?
    public void init(){
        launcherVel.add(-0.01, 0.0);
        launcherVel.add(0.0, 0.0);
        launcherVel.add(4.29, 1267.0);
        launcherVel.add(4.76, 1367.0);
        launcherVel.add(5.22, 1500.0);
        launcherVel.add(5.65, 1667.0);
        launcherVel.add(6.06, 1810.0);
        launcherVel.add(6.48, 2000.0);
        launcherVel.add(10.0, 2100.0);
        launcherVel.createLUT();
        //flywheelController.setTolerance(41);
    }

    public void shoot(int speed){
        //
        if(robot.shooterL.getVelocity() > speed){
            robot.shooterL.setVelocity(robot.shooterL.getVelocity());
            robot.shooterR.setVelocity(robot.shooterR.getVelocity());
            shooterReady = true;
        }else{
//            robot.leftShooter.setVelocity(1300);
//            robot.rightShooter.setVelocity(1300);
            robot.shooterL.set(1);
            robot.shooterR.set(1);
            shooterReady = false;
        }
    }

    public void stop(){
        robot.shooterL.setVelocity(0);robot.shooterR.setVelocity(0);
    }

    public void reverse(){
        robot.shooterL.set(-1);
        robot.shooterR.set(-1);
    }

    //periodic runs in a loop
    @Override
    public void periodic(){
        //updateFlywheel();
    }
}
