package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;
@TeleOp(name = "lets do something cool OP")
public class investingoodtests extends CommandOpMode{
    public GamepadEx driver, driver2;

    public ElapsedTime gameTimer;
    private MecanumDrive drive;

    double speed = 1;
    public ElapsedTime elapsedtime;
    private final Robot robot = Robot.getInstance();

    @Override
    public void initialize(){
        opModeType = OpModeType.TELEOP;

        // DO NOT REMOVE! Resetting FTCLib Command Sechduler
        super.reset();

        robot.init(hardwareMap);
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();

        robot.stopperServo.set(0.45);
        robot.rampServo.set(0.55);


        driver = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        drive = new MecanumDrive(robot.leftFront, robot.rightFront, robot.leftRear, robot.rightRear);

        /// IF THERE NEEDS TO BE MOVEMENT DURING INIT STAGE, UNCOMMENT
        //robot.initHasMovement();

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.stopperServo.set(0.1)),
                    new InstantCommand(() -> robot.intake.start()),
                new InstantCommand(() -> robot.shooter.reverse()))
        );
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
                new SequentialCommandGroup(
                        new InstantCommand(() -> robot.intake.slow()),
                        new InstantCommand(() -> robot.shooter.stop()))
        );

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(
                new InstantCommand(() -> robot.intake.reverse())
        );
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                new InstantCommand(() -> robot.intake.slow())
        );

        driver2.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> Globe.shot = false)
                )
        );
        driver2.getGamepadButton(GamepadKeys.Button.TRIANGLE).whileHeld(

                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.rampServo.set(0.55))
                        //new littleoutshootautofarrr()
                )
        );
        driver2.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> Globe.shot = false)
                )
        );
        driver2.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.shooter.stop()),
                        new InstantCommand(() -> robot.shooter.stop()),
                        new InstantCommand(() -> robot.intake.slow())
                )
        );
        driver2.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> Globe.shot = false)
                )
        );
        driver2.getGamepadButton(GamepadKeys.Button.CIRCLE).whileHeld(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.rampServo.set(0.55))
                        //new littleoutshoot()
                )
        );

        driver2.getGamepadButton(GamepadKeys.Button.CIRCLE).whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.shooter.stop()),
                        new InstantCommand(() -> robot.intake.slow())
                )
        );



        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> Globe.shot = false)
                )
        );
        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whileHeld(

                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.rampServo.set(0.75))
                        //new littleoutshootautofarrr()
                )
        );
        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> Globe.shot = false)
                )
        );
        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.shooter.stop()),
                        new InstantCommand(() -> robot.shooter.stop()),
                        new InstantCommand(() -> robot.intake.slow())
                )
        );
        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new ParallelCommandGroup(
                        new InstantCommand(() -> Globe.shot = false)
                )
        );
        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whileHeld(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.rampServo.set(0.55))
                        //new littleoutshoot()
                )
        );

        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenReleased(
                new ParallelCommandGroup(
                        new InstantCommand(() -> robot.shooter.stop()),
                        new InstantCommand(() -> robot.intake.slow())
                )
        );




        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> robot.rampServo.set(0.55))
        );

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> robot.rampServo.set(robot.rampServo.get()+0.1))
        );

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> robot.rampServo.set(robot.rampServo.get()-0.1))
        );

        if(robot.shooterL.getVelocity() > 100){

            robot.light.setPosition(0.5);//green
        }else{
            robot.light.setPosition(0.28); //red
        }


        super.run();
    }

    @Override
    public void run() {
        // Keep all the has movement init for until when TeleOp starts
        // This is like the init but when the program is actually started
        if (gameTimer == null) {
            robot.initHasMovement();
            gameTimer = new ElapsedTime();
        }

        // DO NOT REMOVE! Runs FTCLib Command Scheudler
        super.run();

        drive.driveRobotCentric(
                driver.getLeftX(),
                driver.getLeftY(),
                driver.getRightX()
        );
        new InstantCommand(() -> robot.rampServo.set(robot.rampServo.get()));

        telemetry.addData("Status", "Running");
        //telemetry.addData("loop times", elapsedtime.milliseconds());
        telemetry.addData("servo", robot.rampServo.get());
        telemetry.addData("stopper", robot.stopperServo.get());
        telemetry.addData("motor speed", robot.shooterL.getVelocity());
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
