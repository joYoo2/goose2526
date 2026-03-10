package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.APRIL_TAG_BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.APRIL_TAG_RED_GOAL;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.LIMELIGHT_PIPELINE;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.TURRET_DEADBAND_DEG;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.TURRET_SPEED;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Turret;

/**
 * Standalone turret PID tuning opmode.
 *
 * Controls:
 *   A / B / X        — select gain to edit: KP / KI / KD
 *   DPAD UP / DOWN   — increase / decrease selected gain
 *   Y                — zero the integral accumulator
 *   DPAD L / R       — manual turret drive (overrides auto-aim while held)
 *
 * Telemetry shows tx, output power, and all three gains live.
 * Changes here update Turret.KP/KI/KD which persist for the rest of the session.
 * Write the final values into Globe.java or Turret.java defaults when satisfied.
 */
@TeleOp(name = "Turret PID Tuning", group = "Test")
public class TurretPIDTuning extends LinearOpMode {

    private enum Gain { KP, KI, KD }

    // Step sizes per button press
    private static final double STEP_KP = 0.002;
    private static final double STEP_KI = 0.001;
    private static final double STEP_KD = 0.9;

    @Override
    public void runOpMode() {
        CRServo turretLeft  = hardwareMap.get(CRServo.class, "turretLeft");
        CRServo turretRight = hardwareMap.get(CRServo.class, "turretRight");

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        Gain selected = Gain.KP;
        ElapsedTime pidTimer  = new ElapsedTime();
        ElapsedTime btnTimer  = new ElapsedTime(); // debounce

        double integral = 0.0;
        double prevTx   = 0.0;
        double output   = 0.0;

        // Button edge detection
        boolean prevA = false, prevB = false, prevX = false, prevY = false;
        boolean prevUp = false, prevDown = false;

        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {
            // ── Gain selection (edge-triggered) ──────────────────────────────
            if (gamepad1.a && !prevA) selected = Gain.KP;
            if (gamepad1.b && !prevB) selected = Gain.KI;
            if (gamepad1.x && !prevX) selected = Gain.KD;
            prevA = gamepad1.a; prevB = gamepad1.b; prevX = gamepad1.x;

            // ── Adjust selected gain (edge-triggered) ─────────────────────────
            if (gamepad1.dpad_up && !prevUp) {
                switch (selected) {
                    case KP: Turret.KP = Math.max(0, Turret.KP + STEP_KP); break;
                    case KI: Turret.KI = Math.max(0, Turret.KI + STEP_KI); break;
                    case KD: Turret.KD = Math.max(0, Turret.KD + STEP_KD); break;
                }
            }
            if (gamepad1.dpad_down && !prevDown) {
                switch (selected) {
                    case KP: Turret.KP = Math.max(0, Turret.KP - STEP_KP); break;
                    case KI: Turret.KI = Math.max(0, Turret.KI - STEP_KI); break;
                    case KD: Turret.KD = Math.max(0, Turret.KD - STEP_KD); break;
                }
            }
            prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;

            // ── Y: zero integral ──────────────────────────────────────────────
            if (gamepad1.y && !prevY) { integral = 0.0; prevTx = 0.0; }
            prevY = gamepad1.y;

            // ── Manual override while dpad left/right held ────────────────────
            if (gamepad1.dpad_left) {
                turretLeft.setPower(-TURRET_SPEED);
                turretRight.setPower(-TURRET_SPEED);
                integral = 0.0; prevTx = 0.0; pidTimer.reset();
                output = -TURRET_SPEED;
            } else if (gamepad1.dpad_right) {
                turretLeft.setPower(TURRET_SPEED);
                turretRight.setPower(TURRET_SPEED);
                integral = 0.0; prevTx = 0.0; pidTimer.reset();
                output = TURRET_SPEED;
            } else {
                // ── Auto-aim PID ──────────────────────────────────────────────
                LLResult result = limelight.getLatestResult();
                boolean tagDetected = false;
                double tx = 0.0;

                if (result != null && result.isValid()) {
                    for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                        int id = tag.getFiducialId();
                        if (id == APRIL_TAG_RED_GOAL || id == APRIL_TAG_BLUE_GOAL) {
                            tagDetected = true;
                            tx = tag.getTargetXDegrees();
                            break;
                        }
                    }
                }

                if (tagDetected && Math.abs(tx) >= TURRET_DEADBAND_DEG) {
                    double dt = pidTimer.seconds();
                    pidTimer.reset();
                    if (dt <= 0) dt = 0.02;

                    double derivative = (tx - prevTx) / dt;
                    integral += tx * dt;
                    integral = Math.max(-20.0, Math.min(20.0, integral));

                    output = Turret.KP * tx + Turret.KI * integral + Turret.KD * derivative;
                    output = Math.max(-TURRET_SPEED, Math.min(TURRET_SPEED, output));
                    turretLeft.setPower(output);
                    turretRight.setPower(output);
                    prevTx = tx;
                } else {
                    turretLeft.setPower(0);
                    turretRight.setPower(0);
                    integral = 0.0; prevTx = 0.0; pidTimer.reset();
                    output = 0.0;
                }

                // Update telemetry tx from last result
                LLResult r2 = limelight.getLatestResult();
                if (r2 != null && r2.isValid() && !r2.getFiducialResults().isEmpty()) {
                    tx = r2.getFiducialResults().get(0).getTargetXDegrees();
                }

                telemetry.addData("tx", "%.2f deg", tx);
                telemetry.addData("Tag", tagDetected ? "YES" : "NO");
            }

            telemetry.addLine("─── PID Gains ───────────────────");
            telemetry.addData(selected == Gain.KP ? "► KP" : "  KP", "%.4f", Turret.KP);
            telemetry.addData(selected == Gain.KI ? "► KI" : "  KI", "%.4f", Turret.KI);
            telemetry.addData(selected == Gain.KD ? "► KD" : "  KD", "%.4f", Turret.KD);
            telemetry.addData("Output", "%.3f", output);
            telemetry.addData("Integral", "%.3f", integral);
            telemetry.addLine("─── Controls ────────────────────");
            telemetry.addLine("A/B/X: select KP/KI/KD");
            telemetry.addLine("DPAD U/D: adjust  |  Y: zero integral");
            telemetry.addLine("DPAD L/R: manual drive");
            telemetry.update();
        }

        limelight.stop();
    }
}
