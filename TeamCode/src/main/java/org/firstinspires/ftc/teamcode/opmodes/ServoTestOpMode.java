package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(name = "Servo Test", group = "Test")
public class ServoTestOpMode extends LinearOpMode {
    boolean shooting;
    @Override
    public void runOpMode() {
        //MotorEx shoot = new MotorEx(hardwareMap, "shoot", Motor.GoBILDA.BARE);
        //shoot.setRunMode(Motor.RunMode.RawPower);
        //shoot.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        //Servo left = hardwareMap.get(Servo.class, "left");
        //Servo right = hardwareMap.get(Servo.class, "right");

        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Initialized. Press Start.");
        telemetry.update();

        waitForStart();

        shooting=false;

        while (opModeIsActive()) {
            //right.setPosition(gamepad1.left_trigger);

            //left.setPosition(gamepad1.left_trigger);

            if (gamepad1.circleWasPressed()) {
                shooting = !shooting;
            }
            if (shooting) {
                shooter1.setPower(.1);
                shooter2.setPower(.1);
            } else {
                shooter1.setPower(0);
                shooter2.setPower(0);
            }
            telemetry.addData("Shooting:",shooting ? "ON" : "OFF");


            //shoot.set(gamepad1.circle ? 1.0 : 0.0);

            //telemetry.addData("Left Servo", "%.2f", gamepad1.left_trigger);
            //telemetry.addData("Right Servo", "%.2f", gamepad1.right_trigger);
            //telemetry.addData("Shooter", gamepad1.circle ? "ON" : "OFF");
            telemetry.update();
        }
    }
}