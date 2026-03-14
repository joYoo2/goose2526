package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Turret;

@TeleOp(name = "Turret Test 90", group = "Test")
public class TurretTest extends LinearOpMode {

    private static final double TARGET_DEG = 90.0;

    @Override
    public void runOpMode() {
        Globe.opModeType = Globe.OpModeType.TELEOP;
        Robot robot = Robot.getInstance();
        robot.init(hardwareMap);

        telemetry.addLine("Press START — turret will drive to 90 deg");
        telemetry.addLine("Hold B to drive to -90  |  default is +90");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double target  = gamepad1.b ? -TARGET_DEG : TARGET_DEG;
            double current = robot.turret.getTurretDegrees();
            double error   = target - current;

            double power = Turret.KP * error;
            power = Math.max(-Globe.TURRET_SPEED, Math.min(Globe.TURRET_SPEED, power));
            if (Math.abs(error) < Globe.TURRET_DEADBAND_DEG) {
                power = 0;
            } else if (Math.abs(power) < Turret.MIN_POWER) {
                power = Math.copySign(Turret.MIN_POWER, power);
            }

            robot.turretLeft.setPower(Turret.DIRECTION * power);
            robot.turretRight.setPower(Turret.DIRECTION * power);

            // Keep encoder tracking up to date
            robot.turret.periodic();

            telemetry.addData("Target",  "%.1f deg", target);
            telemetry.addData("Current", "%.1f deg", current);
            telemetry.addData("Error",   "%.1f deg", error);
            telemetry.addData("Power",   "%.3f", power);
            telemetry.addData("", "Hold B for -90, default is +90");
            telemetry.update();
        }

        robot.turretLeft.setPower(0);
        robot.turretRight.setPower(0);
    }
}