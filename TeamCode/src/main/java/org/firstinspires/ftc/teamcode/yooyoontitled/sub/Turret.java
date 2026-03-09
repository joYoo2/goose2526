package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

import java.util.List;

public class Turret extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum AimMode { MANUAL, AUTO_AIM }

    private AimMode mode = AimMode.MANUAL;

    // Future use: offset in degrees from tag center (e.g. for angled shots or velocity lead)
    private double aimOffset = 0.0;

    // Soft limit: accumulates power * dt each loop, clamped to ±TURRET_SOFT_LIMIT
    private double turretPosition = 0.0;
    private final ElapsedTime timer = new ElapsedTime();

    public AimMode getMode() {
        return mode;
    }

    public void setMode(AimMode mode) {
        this.mode = mode;
        if (mode == AimMode.MANUAL) applyPower(0);
    }

    public void toggleMode() {
        setMode(mode == AimMode.AUTO_AIM ? AimMode.MANUAL : AimMode.AUTO_AIM);
    }

    public void setAimOffset(double degrees) {
        aimOffset = degrees;
    }

    // Manual control — always available regardless of mode
    public void turnLeft()  { applyPower(-TURRET_SPEED); }
    public void turnRight() { applyPower(TURRET_SPEED);  }
    public void stop()      { applyPower(0);             }

    public double getTurretPosition() { return turretPosition; }

    private void applyPower(double power) {
        double dt = timer.seconds();
        timer.reset();

        // Soft limit: prevent winding past safe range
        if (turretPosition >= TURRET_SOFT_LIMIT  && power > 0) power = 0;
        if (turretPosition <= -TURRET_SOFT_LIMIT && power < 0) power = 0;

        turretPosition += power * dt;

        robot.turretLeft.setPower(power);
        robot.turretRight.setPower(power);
    }

    @Override
    public void periodic() {
        if (mode != AimMode.AUTO_AIM) return;

        LLResult result = robot.limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            applyPower(0);
            return;
        }

        int targetId = (Globe.goalColor == Globe.GoalColor.BLUE_GOAL)
                ? APRIL_TAG_BLUE_GOAL : APRIL_TAG_RED_GOAL;

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        LLResultTypes.FiducialResult target = null;
        for (LLResultTypes.FiducialResult tag : tags) {
            if (tag.getFiducialId() == targetId) {
                target = tag;
                break;
            }
        }

        if (target == null) {
            applyPower(0);
            return;
        }

        double tx = target.getTargetXDegrees() + aimOffset;

        if (Math.abs(tx) < TURRET_DEADBAND_DEG) {
            applyPower(0);
            return;
        }

        double power = TURRET_KP * tx;
        power = Math.max(-TURRET_SPEED, Math.min(TURRET_SPEED, power));
        applyPower(power);
    }
}