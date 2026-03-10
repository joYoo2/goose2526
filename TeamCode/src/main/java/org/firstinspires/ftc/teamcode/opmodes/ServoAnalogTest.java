package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

/**
 * Axon Mini analog feedback test — multi-turn tracking.
 *
 * HOW TO USE:
 *   1. Drive the servo back and forth through its full range.
 *      "Observed min/max" stabilizes; volts/turn is derived automatically.
 *   2. Press Square to zero the turn counter at your reference position.
 *   3. Press Cross to lock current position as hold target.
 *      Press DPAD U/D to nudge the target ±0.1 turns.
 *   4. Turns here = servo shaft turns. Map to turret degrees later
 *      using your gear ratio: turret_deg = turns * (360 / gear_ratio).
 *
 * CONTROLS:
 *   DPAD L/R   — manual drive (hold resumes when released)
 *   Cross      — save current position as waypoint
 *   Circle     — go to saved waypoint and hold
 *   Square     — zero here (Turns resets to 0.00, clears waypoint)
 *   Triangle   — log current position to telemetry
 *   DPAD U/D   — nudge saved waypoint ±0.1 turns
 */
@TeleOp(name = "Servo Analog Test", group = "Test")
public class ServoAnalogTest extends LinearOpMode {

    static final String SERVO_NAME   = "turretLeft";
    static final String SERVO2_NAME  = "turretRight";
    static final String ANALOG_NAME  = "turretEncoder";

    static final double MANUAL_SPEED  = 0.4;
    static final double HOLD_SPEED    = 0.4;
    static final double HOLD_KP       = 2.0;
    static final double DEADBAND_VOLT = 0.01;
    static final double NUDGE_TURNS   = 0.1;
    // Flip to -1.0 if hold drives the wrong direction (servo voltage anti-correlates with power)
    static final double DIRECTION     = -1.0;

    @Override
    public void runOpMode() {
        CRServo     servo    = hardwareMap.get(CRServo.class,    SERVO_NAME);
        CRServo     servo2   = hardwareMap.get(CRServo.class,    SERVO2_NAME);
        AnalogInput feedback = hardwareMap.get(AnalogInput.class, ANALOG_NAME);

        waitForStart();

        // ── Observed range (auto-discovered by driving around) ────────────────
        // seenMax - seenMin = volts per servo shaft turn, by definition.
        double seenMin = Double.MAX_VALUE;
        double seenMax = -Double.MAX_VALUE;

        // ── Multi-turn tracking ───────────────────────────────────────────────
        double prevVoltage  = feedback.getVoltage();
        int    turnCount    = 0;
        double zeroOffset   = 0.0;   // rawTotal at the user's zero reference
        double savedTarget  = Double.NaN;  // waypoint saved by Cross
        boolean holding     = false;       // true = actively driving to savedTarget

        int waypointNum = 1;

        // ── Previous button states ────────────────────────────────────────────
        boolean prevCross = false, prevCircle = false, prevSquare = false;
        boolean prevTriangle = false, prevUp = false, prevDown = false;

        while (opModeIsActive()) {
            double voltage = feedback.getVoltage();

            // ── Auto min/max → volts-per-turn ─────────────────────────────────
            seenMin = Math.min(seenMin, voltage);
            seenMax = Math.max(seenMax, voltage);
            double voltsPerTurn = seenMax - seenMin;   // valid once range stabilizes
            boolean calibrated  = voltsPerTurn > 0.1;  // ignore noise before driving

            // ── Wrap detection ────────────────────────────────────────────────
            if (calibrated) {
                double delta     = voltage - prevVoltage;
                double threshold = voltsPerTurn / 2.0;
                if      (delta >  threshold) turnCount--;
                else if (delta < -threshold) turnCount++;
            }
            double rawTotal     = turnCount * (calibrated ? voltsPerTurn : 3.3) + voltage;
            double displayTotal = rawTotal - zeroOffset;
            double displayTurns = calibrated ? displayTotal / voltsPerTurn : Double.NaN;
            prevVoltage = voltage;

            // ── Button edges ──────────────────────────────────────────────────
            boolean pressCross    = gamepad1.cross     && !prevCross;
            boolean pressCircle   = gamepad1.circle    && !prevCircle;
            boolean pressSquare   = gamepad1.square    && !prevSquare;
            boolean pressTriangle = gamepad1.triangle  && !prevTriangle;
            boolean pressUp       = gamepad1.dpad_up   && !prevUp;
            boolean pressDown     = gamepad1.dpad_down && !prevDown;
            prevCross    = gamepad1.cross;
            prevCircle   = gamepad1.circle;
            prevSquare   = gamepad1.square;
            prevTriangle = gamepad1.triangle;
            prevUp       = gamepad1.dpad_up;
            prevDown     = gamepad1.dpad_down;

            if (pressCross)  { savedTarget = rawTotal; holding = false; }  // save, don't hold yet
            if (pressCircle && !Double.isNaN(savedTarget)) holding = true;  // go to saved waypoint
            if (pressSquare) { zeroOffset = rawTotal; savedTarget = Double.NaN; holding = false; }

            if (pressUp   && !Double.isNaN(savedTarget))
                savedTarget += calibrated ? NUDGE_TURNS * voltsPerTurn : NUDGE_TURNS;
            if (pressDown && !Double.isNaN(savedTarget))
                savedTarget -= calibrated ? NUDGE_TURNS * voltsPerTurn : NUDGE_TURNS;

            if (pressTriangle) {
                if (!Double.isNaN(displayTurns)) {
                    telemetry.addData(">> Waypoint " + waypointNum,
                            "%.3f turns", displayTurns);
                } else {
                    telemetry.addData(">> Waypoint " + waypointNum,
                            "%.4f V  (drive more to calibrate)", voltage);
                }
                waypointNum++;
            }

            // ── Drive ─────────────────────────────────────────────────────────
            double power = 0.0;
            if (gamepad1.dpad_right) {
                power = MANUAL_SPEED;           // manual overrides hold temporarily
            } else if (gamepad1.dpad_left) {
                power = -MANUAL_SPEED;
            } else if (holding && !Double.isNaN(savedTarget)) {
                double error = savedTarget - rawTotal;
                if (Math.abs(error) > DEADBAND_VOLT) {
                    power = Math.max(-HOLD_SPEED, Math.min(HOLD_SPEED, HOLD_KP * DIRECTION * error));
                }
            }
            servo.setPower(power);
            servo2.setPower(power);

            // ── Telemetry ─────────────────────────────────────────────────────
            telemetry.addLine("── Raw ──────────────────────────────");
            telemetry.addData("Voltage",     "%.4f V", voltage);
            telemetry.addData("Observed",    "min=%.4f  max=%.4f", seenMin, seenMax);
            telemetry.addData("Volts/turn",  calibrated
                    ? String.format("%.4f V  (drive more to widen)", voltsPerTurn)
                    : "-- drive servo through full range --");

            telemetry.addLine("── Position ─────────────────────────");
            if (!Double.isNaN(displayTurns)) {
                telemetry.addData("Turns", "%.3f", displayTurns);
            } else {
                telemetry.addData("Turns", "uncalibrated — drive more");
            }
            if (!Double.isNaN(savedTarget)) {
                double targetDisplay = savedTarget - zeroOffset;
                double errorTotal    = savedTarget - rawTotal;
                String holdStr       = holding ? "HOLDING" : "saved (Circle to go)";
                if (calibrated) {
                    telemetry.addData("Waypoint", "%.3f turns  [%s]", targetDisplay / voltsPerTurn, holdStr);
                    telemetry.addData("Error",    "%.3f turns", errorTotal / voltsPerTurn);
                } else {
                    telemetry.addData("Waypoint", "%.4f V  [%s]", targetDisplay, holdStr);
                    telemetry.addData("Error",    "%.4f V", errorTotal);
                }
            } else {
                telemetry.addLine("Waypoint: NONE  (Cross to save)");
            }
            telemetry.addData("Output", "%.3f", power);

            telemetry.addLine("── Controls ─────────────────────────");
            telemetry.addLine("DPAD L/R: drive      Cross: save waypoint");
            telemetry.addLine("Circle: go to waypoint  Square: zero here");
            telemetry.addLine("Triangle: log pos    DPAD U/D: nudge 0.1t");
            telemetry.update();
        }
    }
}
