package org.firstinspires.ftc.teamcode.opmodes;



import com.bylazar.gamepad.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;
import org.firstinspires.ftc.teamcode.yooyoontitled.commands.AutoShoot;
import org.firstinspires.ftc.teamcode.yooyoontitled.commands.littleoutshoot;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;


@Autonomous(name = "our leave auton")
public class reallyreallyauto extends LinearOpMode {

    //made by adhithya ;D
    public DcMotor leftFront, leftRear, rightRear, rightFront;

    public void runOpMode() throws InterruptedException{
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        if (opModeIsActive()){
            sleep(29000);
            leftFront.setPower(-1);
            leftRear.setPower(-1);
            rightFront.setPower(-1);
            rightRear.setPower(-1);
            sleep(1000);
        }
    }
/*
    private MecanumDrive drive;

    double speed = 1;
    public ElapsedTime elapsedtime;
    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize(){
        opModeType = OpModeType.AUTO;
        super.reset();

        robot.init(hardwareMap);
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
        drive = new MecanumDrive(robot.leftFront, robot.rightFront, robot.leftRear, robot.rightRear);

    }

    @Override
    public void run() {
        super.run();
        wait(10000);
    }
*/

}
