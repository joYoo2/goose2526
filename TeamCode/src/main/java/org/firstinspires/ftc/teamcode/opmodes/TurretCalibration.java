package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Turret Calibration", group = "Test")
public class TurretCalibration extends LinearOpMode {

    @Override
    public void runOpMode() {
        CRServo turretLeft  = hardwareMap.get(CRServo.class, "turretLeft");
        CRServo turretRight = hardwareMap.get(CRServo.class, "turretRight");

        waitForStart();

        ElapsedTime timer = new ElapsedTime();
        double turretPosition = 0.0;

        while (opModeIsActive()) {
            double dt = timer.seconds();
            timer.reset();

            boolean dpadLeft  = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;
            boolean btnY      = gamepad1.y;

            if (dpadLeft) {
                turretLeft.setPower(-1);
                turretRight.setPower(-1);
                turretPosition += -dt;
            } else if (dpadRight) {
                turretLeft.setPower(1);
                turretRight.setPower(1);
                turretPosition += dt;
            } else {
                turretLeft.setPower(0);
                turretRight.setPower(0);
            }

            if (btnY) {
                turretPosition = 0.0;
            }

            telemetry.addData("Turret Position (accum)", "%.4f", turretPosition);
            telemetry.addLine("DPAD L/R: drive | Y: zero here");
            telemetry.addLine("Drive to wire limit, note the position value, set TURRET_SOFT_LIMIT to that value");
            telemetry.update();
        }
    }
}
