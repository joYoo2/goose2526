package org.firstinspires.ftc.teamcode.opmodes;



import com.bylazar.gamepad.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.RepeatCommand;
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
import org.firstinspires.ftc.teamcode.yooyoontitled.commands.littleoutshootauto;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;


@Autonomous(name = "autontestingcouldbebad")
public class autononooonnfar extends CommandOpMode{
    public ElapsedTime gameTimer;
    private MecanumDrive drive;
    public ElapsedTime elapsedtime;
    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;

        elapsedtime = new ElapsedTime();
        super.reset();

        robot.init(hardwareMap);

        robot.stopperServo.set(0.45);
        robot.rampServo.set(0.55);

        register(robot.shooter);
//
        drive = new MecanumDrive(robot.leftFront, robot.rightFront, robot.leftRear, robot.rightRear);
        drive.driveRobotCentric(0,0,0);

        schedule(
                new SequentialCommandGroup(
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new InstantCommand((() -> robot.rampServo.set(0.55))),
                                new InstantCommand(() -> drive.driveRobotCentric(0,-0.50,0)),
                                new InstantCommand(() -> robot.intake.set(0.1))
                        ),

                        new WaitCommand(500),
                        new InstantCommand(() -> drive.driveRobotCentric(0,0,0)),
                        new WaitCommand(500),
                        new RepeatCommand(
                                new littleoutshootauto()
                        ).withTimeout(2000),
                        new WaitCommand(2100),
                        new InstantCommand(() -> robot.shooterMotor.set(0)),
                        new WaitCommand(200),
                        new InstantCommand(() -> robot.intake.set(0)),
                        new WaitCommand(200),
                        new InstantCommand(() -> drive.driveRobotCentric(-1,0,0)),
                        new WaitCommand(500),
                        new InstantCommand(() -> drive.driveRobotCentric(0,0,0))
                )
        );

    }

    @Override
    public void initialize_loop(){

        //telemetry.addData("randomization:", randomizationMotif.toString());
        telemetry.update();
        //drawOnlyCurrent();
    }

    @Override
    public void run() {

        // DO NOT REMOVE! Runs FTCLib Command Scheudler
        super.run();
        new InstantCommand(() -> robot.rampServo.set(robot.rampServo.get()));

        telemetry.addData("Status", "Running");
        //telemetry.addData("loop times", elapsedtime.milliseconds());
        telemetry.addData("servo", robot.rampServo.get());
        telemetry.addData("stopper", robot.stopperServo.get());
        telemetry.addData("motor speed", robot.shooterMotor.getVelocity());
        telemetry.addData("digaierg right", shooterReady);
        elapsedtime.reset();

        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        //robot.ControlHub.clearBulkCache();
        for(LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }
    }

}
