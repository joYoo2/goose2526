package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

@TeleOp(name = "Stopper Test", group = "Test")
public class StopperTest extends LinearOpMode {

    private static final double STEP = 0.1;

    @Override
    public void runOpMode() {
        Globe.opModeType = Globe.OpModeType.TELEOP;
        Robot robot = Robot.getInstance();
        robot.init(hardwareMap);

        double pos = 0.5;
        robot.stopperServo.set(pos);

        waitForStart();

        boolean prevLeft  = false;
        boolean prevRight = false;

        while (opModeIsActive()) {
            boolean curLeft  = gamepad1.dpad_left;
            boolean curRight = gamepad1.dpad_right;

            if (curLeft && !prevLeft) {
                pos = Math.max(0.0, pos - STEP);
                robot.stopperServo.set(pos);
            }
            if (curRight && !prevRight) {
                pos = Math.min(1.0, pos + STEP);
                robot.stopperServo.set(pos);
            }

            prevLeft  = curLeft;
            prevRight = curRight;

            telemetry.addData("Position", "%.3f", pos);
            telemetry.addData("Controls", "DPAD LEFT = -0.01  |  DPAD RIGHT = +0.01");
            telemetry.update();
        }
    }
}
