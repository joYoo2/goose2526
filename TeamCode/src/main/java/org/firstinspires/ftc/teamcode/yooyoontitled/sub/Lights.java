package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

public class Lights extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public double RED = 0.28;
    public double ORANGE = 0.33;
    public double YELLOW = 0.39;
    public double SAGE = 0.44;
    public double GREEN = 0.5;
    public double AZURE = 0.555;
    public double BLUE = 0.611;
    public double INDIGO = 0.666;
    public double VIOLET = 0.72;
    public double WHITE = 1;
    public double OFF = 0;

    public double constantColor = ORANGE;

    public void init(){
    }

    public static LightsState lightsState;

    public enum LightsState{
        SHOOTER_READY,
        TEAM_COLOR,
        CONSTANT_COLOR
    }

    public void updateLights(){
        if(lightsState == LightsState.SHOOTER_READY){
            shooterReady();
        }

        if(lightsState == LightsState.TEAM_COLOR){
            teamColor();
        }

        if(lightsState == LightsState.CONSTANT_COLOR){
            /// WHEN USING CONSTANT COLOR MAKE SURE TO CHANGE CONSTANT COLOR VARIABLE
            constantColor(constantColor);
        }
    }

    /**
     * Shows if shooter is ready to shoot
     * RED = not ready
     * GREEN = ready to shoot
     */
    public void shooterReady(){
        if(!shooterReady){
            robot.lightning.setPosition(RED);
        }else{
            robot.lightning.setPosition(GREEN);
        }
    }

    /**
     * Shows the team color
     * RED = Red alliance
     * BLUE = Blue alliance
     */
    public void teamColor(){
        if(goals == GoalColor.RED_GOAL){
            robot.lightning.setPosition(RED);
        }else{
            robot.lightning.setPosition(BLUE);
        }
    }

    public void constantColor(double color){
        robot.lightning.setPosition(color);
    }

    //periodic runs in a loop
    @Override
    public void periodic(){
        updateLights();
    }
}