package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(name = "Servo Test", group = "Test")
public class ServoTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        //MotorEx shoot = new MotorEx(hardwareMap, "shoot", Motor.GoBILDA.BARE);
        //shoot.setRunMode(Motor.RunMode.RawPower);
        //shoot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        Servo left = hardwareMap.get(Servo.class, "left");
        Servo right = hardwareMap.get(Servo.class, "right");

        telemetry.addLine("Initialized. Press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            right.setPosition(gamepad1.left_trigger);

            left.setPosition(gamepad1.left_trigger);


            //shoot.set(gamepad1.circle ? 1.0 : 0.0);

            telemetry.addData("Left Servo", "%.2f", gamepad1.left_trigger);
            telemetry.addData("Right Servo", "%.2f", gamepad1.right_trigger);
            //telemetry.addData("Shooter", gamepad1.circle ? "ON" : "OFF");
            telemetry.update();
        }
    }
}