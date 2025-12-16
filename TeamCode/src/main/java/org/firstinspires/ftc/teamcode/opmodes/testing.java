package org.firstinspires.ftc.teamcode.opmodes;


import com.bylazar.gamepad.*;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;
import org.firstinspires.ftc.teamcode.yooyoontitled.commands.AutoShoot;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

@TeleOp(name = "testing")
public class testing extends CommandOpMode{
    public GamepadEx driver, driver2;
    public int speed;
    private MecanumDrive drive;
    public ElapsedTime elapsedtime;
    public ElapsedTime gameTimer;
    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize(){
        opModeType = OpModeType.TELEOP;
        // DO NOT REMOVE! Resetting FTCLib Command Sechduler
        super.reset();
        speed = 1600;

        robot.init(hardwareMap);
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
        robot.stopperServo.set(0.45);

        register(robot.intake,robot.shooter);
        driver = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        drive = new MecanumDrive(robot.leftFront, robot.rightFront, robot.leftRear, robot.rightRear);

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.stopperServo.set(0.1)),
                        new InstantCommand(() -> robot.intake.start()),
                        new InstantCommand(() -> robot.shooter.reverse()))

        );
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.intake.slow()),
                        new InstantCommand(() -> robot.shooter.stop()))

        );

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(
                new InstantCommand(() -> robot.intake.reverse())

        );
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
                new InstantCommand(() -> robot.intake.slow())

        );

        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.stopperServo.set(0.1)),
                        new InstantCommand(() -> robot.intake.start()),
                        new InstantCommand(() -> robot.shooter.reverse()))

        );
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.intake.slow()),
                        new InstantCommand(() -> robot.shooter.stop()))

        );

        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(
                new InstantCommand(() -> robot.intake.reverse())

        );
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
                new InstantCommand(() -> robot.intake.slow())

        );

        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whileHeld(
                new AutoShoot(1600)
        );

        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.shooter.stop()),
                        new InstantCommand(() -> robot.intake.slow())
                )

        );

        // Shooter controls commented out - motors not plugged in
        driver2.getGamepadButton(GamepadKeys.Button.TRIANGLE).whileHeld(
                new AutoShoot(1600)
        );

        driver2.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.shooter.stop()),
                        new InstantCommand(() -> robot.intake.slow())
                )

        );

        driver2.getGamepadButton(GamepadKeys.Button.CIRCLE).whileHeld(
                new AutoShoot(speed)
        );

        driver2.getGamepadButton(GamepadKeys.Button.CIRCLE).whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.shooter.stop()),
                        new InstantCommand(() -> robot.intake.slow())
                )

        );

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(()-> speed += 50)
        );

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(()-> speed -= 50)
        );

        driver2.getGamepadButton(GamepadKeys.Button.CIRCLE).whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.shooter.stop()),
                        new InstantCommand(() -> robot.intake.slow())
                )

        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(

                new InstantCommand(() -> robot.stopperServo.set(robot.stopperServo.get() - 0.05))

        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(

                new InstantCommand(() -> robot.stopperServo.set(robot.stopperServo.get() + 0.05))

        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(

                new InstantCommand(() -> robot.stopperServo.set(0.1))

        );

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(

                new InstantCommand(() -> robot.stopperServo.set(0.45))

        );




        super.run();


    }

    @Override
    public void run() {
        if (gameTimer == null) {
            robot.initHasMovement();

            gameTimer = new ElapsedTime();
        }
        super.run();


        drive.driveRobotCentric(
                driver.getLeftX(),
                driver.getLeftY(),
                driver.getRightX()
        );
        new InstantCommand(() -> robot.rampServo.set(robot.rampServo.get()));

        telemetry.addData("Status", "Running");
        telemetry.addData("speed", speed);
        //telemetry.addData("loop times", elapsedtime.milliseconds());
        telemetry.addData("stopper", robot.stopperServo.get());
//         Shooter telemetry commented out - motors not plugged in
        telemetry.addData("motor speed", robot.shooter1.getVelocity());
        telemetry.addData("digaierg right", shooterReady);
        elapsedtime.reset();

        telemetry.update();

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
