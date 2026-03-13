package org.firstinspires.ftc.teamcode.yooyoontitled.sub;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;
import org.firstinspires.ftc.teamcode.yooyoontitled.ShootingUtils;

public class Turret extends SubsystemBase {
    private final Robot robot = Robot.getInstance();

    public enum AimMode { MANUAL, AUTO_AIM }

    private AimMode mode = AimMode.MANUAL;

    // Encoder constants
    static final double VOLTS_PER_TURN = 3.2;
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
    public static double KD = 0.0; // zeroed: derivative on noisy analog+pose signal causes oscillation

    // Anti-shake: power below this threshold is rounded to zero
    public static final double MIN_POWER = 0.10;
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

    public AimMode getMode() { return mode; }

    public void setMode(AimMode mode) {
        this.mode = mode;
        if (mode == AimMode.MANUAL) {
            resetPID();
            stop();
        }
    }

    public void toggleMode() {
        setMode(mode == AimMode.AUTO_AIM ? AimMode.MANUAL : AimMode.AUTO_AIM);
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

        if (mode != AimMode.AUTO_AIM) return;

        // Desired turret heading from robot pose + goal position
        Pose robotPose = robot.follower.getPose();
        double fieldAngle = ShootingUtils.calculateTargetHeading(robotPose, Globe.goalColor);
        double desiredDeg = normalizeAngle(
                Math.toDegrees(fieldAngle - robotPose.getHeading()));
        desiredDeg = Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, desiredDeg));

        double currentDeg = getTurretDegrees();
        double error = desiredDeg - currentDeg;

        lastError = error;

        // Hysteretic deadband — prevents chattering at the boundary
        double enterThreshold = TURRET_DEADBAND_DEG;
        double exitThreshold  = TURRET_DEADBAND_EXIT_DEG;
        if (inDeadband) {
            if (Math.abs(error) > exitThreshold) {
                inDeadband = false;
            } else {
                stop();
                return;
            }
        } else {
            if (Math.abs(error) < enterThreshold) {
                inDeadband = true;
                stop();
                return;
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
