package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

public class Lights extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public static final double RED    = 0.28;
    public static final double ORANGE = 0.33;
    public static final double YELLOW = 0.39;
    public static final double SAGE   = 0.44;
    public static final double GREEN  = 0.5;
    public static final double AZURE  = 0.555;
    public static final double BLUE   = 0.611;
    public static final double INDIGO = 0.666;
    public static final double VIOLET = 0.72;
    public static final double WHITE  = 1.0;
    public static final double OFF    = 0.0;

    public double constantColor = ORANGE;

    public static LightsState lightsState = LightsState.TEAM_COLOR;

    public enum LightsState {
        SHOOTER_READY,
        INTAKE_ACTIVE,
        TEAM_COLOR,
        CONSTANT_COLOR
    }

    /**
     * Green = shooter ready, Red = still spinning up.
     */
    private void shooterReady() {
        robot.lightsServo.setPosition(Globe.shooterReady ? GREEN : RED);
    }

    /**
     * Pulses orange while intake is running.
     */
    private void intakeActive() {
        robot.lightsServo.setPosition(ORANGE);
    }

    /**
     * Shows alliance color.
     */
    private void teamColor() {
        robot.lightsServo.setPosition(goalColor == GoalColor.RED_GOAL ? RED : BLUE);
    }

    private void constantColor(double color) {
        robot.lightsServo.setPosition(color);
    }

    @Override
    public void periodic() {
        switch (lightsState) {
            case SHOOTER_READY:   shooterReady();         break;
            case INTAKE_ACTIVE:   intakeActive();         break;
            case TEAM_COLOR:      teamColor();            break;
            case CONSTANT_COLOR:  constantColor(constantColor); break;
        }
    }
}
