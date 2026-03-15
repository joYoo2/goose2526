package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;
import org.firstinspires.ftc.teamcode.yooyoontitled.ShootingUtils;

public class Turret extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum AimMode { MANUAL, AUTO_AIM, RETURN_TO_ZERO, AIM_AT_POSE }

    private AimMode mode = AimMode.MANUAL;
    private Pose targetPose = null;  // For AIM_AT_POSE mode

    // Encoder constants
    static final double VOLTS_PER_TURN = 3.25;
    static final double TURRET_DEG_PER_SERVO_TURN = 360.0 * 28.0 / 70.0; // = 144.0
    public static final double DIRECTION = -1.0; // flip if turret drives wrong way

    // Encoder tracking
    private double prevVoltage;
    private int turnCount = 0;
    private double zeroVoltageTotal; // rawTotal at init (turret facing forward)
    private double prevError = 0.0;

    // PID gains — public static so FTC Dashboard can tune live
    public static double KP = 0.02;
    public static double KI = 0.0;
    public static double KD = 0.001; // zeroed: derivative on noisy analog+pose signal causes oscillation

    // Anti-shake: power below this threshold is rounded to zero
    public static final double MIN_POWER = 0.05;
    // Hysteresis: once stopped inside deadband, don't restart until error exceeds this
    static final double TURRET_DEADBAND_EXIT_DEG = 3.0;

    // PID state
    private double integral  = 0.0;
    private final ElapsedTime pidTimer = new ElapsedTime();
    private boolean inDeadband = false;

    // Telemetry — read by OpMode / tuning opmode
    public double lastError  = 0.0;
    public double lastOutput = 0.0;

    public Turret() {
        prevVoltage = robot.turretEncoder.getVoltage();
        zeroVoltageTotal = prevVoltage; // turnCount=0 at init
    }

    /** Resets the current turret position to be the new "zero" (forward). */
    public void resetZero() {
        double rawTotal = turnCount * VOLTS_PER_TURN + prevVoltage;
        zeroVoltageTotal = rawTotal;
    }

    public AimMode getMode() { return mode; }

    public void setMode(AimMode mode) {
        this.mode = mode;
        if (mode == AimMode.MANUAL) {
            resetPID();
            stop();
        }
    }

    /**
     * Set turret to aim as if robot is at a specific pose (for autonomous pre-aiming).
     * @param pose The pose to aim from (typically the final shooting position)
     */
    public void aimAtPose(Pose pose) {
        this.targetPose = pose;
        this.mode = AimMode.AIM_AT_POSE;
        resetPID();
    }

    public void toggleMode() {
        if (mode == AimMode.AUTO_AIM) {
            // Disable auto-aim and return to forward position
            resetPID();
            mode = AimMode.RETURN_TO_ZERO;
        } else if (mode == AimMode.MANUAL || mode == AimMode.RETURN_TO_ZERO) {
            resetPID();
            mode = AimMode.AUTO_AIM;
        }
    }

    public void resetPID() {
        integral  = 0.0;
        prevError = 0.0;
        pidTimer.reset();
    }

    public void turnLeft() {
        setPower(-TURRET_SPEED);
    }
    public void turnRight() {
        setPower(TURRET_SPEED);
    }
    public void stop() { setPower(0); }

    private void setPower(double power) {
        lastOutput = power;
        robot.turretLeft.setPower(power);
        robot.turretRight.setPower(power);
    }

    /** Call every loop to track multi-turn wraps. */
    private void updateEncoder() {
        double voltage = robot.turretEncoder.getVoltage();
        double delta = voltage - prevVoltage;
        double threshold = VOLTS_PER_TURN / 2.0;
        if (delta > threshold)       turnCount--;
        else if (delta < -threshold) turnCount++;
        prevVoltage = voltage;
    }

    /** Current turret angle in degrees (0 = forward at init). */
    public double getTurretDegrees() {
        double rawTotal = turnCount * VOLTS_PER_TURN + prevVoltage;
        double servoTurns = (rawTotal - zeroVoltageTotal) / VOLTS_PER_TURN;
        return servoTurns * TURRET_DEG_PER_SERVO_TURN;
    }

    private double normalizeAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) degrees -= 360;
        if (degrees <= -180) degrees += 360;
        return degrees;
    }

    @Override
    public void periodic() {
        updateEncoder(); // always track wraps, even in MANUAL mode

        if (mode == AimMode.MANUAL) return;

        double desiredDeg;
        if (mode == AimMode.RETURN_TO_ZERO) {
            desiredDeg = 0;
        } else if (mode == AimMode.AIM_AT_POSE) {
            // AIM_AT_POSE: aim as if robot is at targetPose (for autonomous pre-aiming)
            if (targetPose == null) {
                // Safety fallback if targetPose not set
                desiredDeg = 0;
            } else {
                double fieldAngle = ShootingUtils.calculateTargetHeading(targetPose, Globe.goalColor);
                desiredDeg = normalizeAngle(Math.toDegrees(fieldAngle - targetPose.getHeading()));
                desiredDeg = Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, desiredDeg));
            }
        } else {
            // AUTO_AIM: desired heading from current robot pose + goal position
            Pose robotPose = robot.follower.getPose();
            double fieldAngle;

            if (Globe.VELOCITY_COMP_MODE) {
                // Velocity compensation: use lead-compensated heading
                Vector velocity = robot.follower.getVelocity();
                double robotVelX = velocity.getXComponent();
                double robotVelY = velocity.dot(new Vector(1, Math.PI / 2));  // Y component via dot product
                fieldAngle = ShootingUtils.calculateLeadHeading(robotPose, robotVelX, robotVelY, Globe.goalColor);
            } else {
                // Standard aiming: aim directly at goal
                fieldAngle = ShootingUtils.calculateTargetHeading(robotPose, Globe.goalColor);
            }

            desiredDeg = normalizeAngle(Math.toDegrees(fieldAngle - robotPose.getHeading()));
            desiredDeg = Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, desiredDeg));
        }

        double currentDeg = getTurretDegrees();
        double error = desiredDeg - currentDeg;

        lastError = error;

        // Velocity comp mode: skip deadband for continuous tracking
        // Standard mode: use hysteretic deadband to prevent chattering
        if (!Globe.VELOCITY_COMP_MODE) {
            double enterThreshold = TURRET_DEADBAND_DEG;
            double exitThreshold  = TURRET_DEADBAND_EXIT_DEG;
            if (inDeadband) {
                if (Math.abs(error) > exitThreshold) {
                    inDeadband = false;
                } else {
                    stop();
                    if (mode == AimMode.RETURN_TO_ZERO) mode = AimMode.MANUAL;
                    return;
                }
            } else {
                if (Math.abs(error) < enterThreshold) {
                    inDeadband = true;
                    stop();
                    if (mode == AimMode.RETURN_TO_ZERO) mode = AimMode.MANUAL;
                    return;
                }
            }
        }

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0) dt = 0.02;

        double derivative = (error - prevError) / dt;
        integral += error * dt;
        integral = Math.max(-20.0, Math.min(20.0, integral));

        double power = KP * error + KI * integral + KD * derivative;
        power = Math.max(-TURRET_SPEED, Math.min(TURRET_SPEED, power));
        if (Math.abs(power) < MIN_POWER) power = 0;
        setPower(DIRECTION * power);

        prevError = error;
    }
}
