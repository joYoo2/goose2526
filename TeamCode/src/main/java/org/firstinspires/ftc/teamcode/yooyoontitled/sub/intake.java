package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

public class Intake extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum IntakeState { IDLE, INTAKING, REVERSED }

    private IntakeState state = IntakeState.IDLE;
    private boolean lowerSpinning = true;

    public void setState(IntakeState state) {
        this.state = state;
    }

    public IntakeState getState() {
        return state;
    }

    public void toggleLowerSpinning() {
        lowerSpinning = !lowerSpinning;
    }

    public void lowerIntake() {
        robot.intakeLiftLeft.set(1-LIFT_LOWERED);
        robot.intakeLiftRight.set(LIFT_LOWERED);
    }

    public void raiseIntake() {
        robot.intakeLiftLeft.set(1-LIFT_RAISED);
        robot.intakeLiftRight.set(LIFT_RAISED);
    }

    @Override
    public void periodic() {
        double lower, upper;

        if (state == IntakeState.REVERSED) {
            lower = INTAKE_REVERSE_SPEED;
            upper = INTAKE_REVERSE_SPEED;
        } else if (state == IntakeState.INTAKING || Globe.shooterReady) {
            lower = INTAKE_LOWER_SPEED;
            upper = Globe.shooterReady ? INTAKE_UPPER_FEED_SPEED : INTAKE_UPPER_PASSIVE_SPEED;

        } else {
            lower = lowerSpinning ? INTAKE_LOWER_SPEED : 0;
            upper = 0;
        }

        robot.intakeLower.set(lower);
        robot.intakeUpper.set(upper);

        // Lift: raised when intaking or reversing, lowered when idle
        if (state == IntakeState.INTAKING || state == IntakeState.REVERSED) {
            raiseIntake();
        } else {
            lowerIntake();
        }

        // Kick servo: forward when shooting/ejecting, reverse when intaking, stopped otherwise

    }
}
