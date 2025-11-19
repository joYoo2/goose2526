package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

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

import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

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

        driver = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);
        drive = new MecanumDrive(robot.leftFront, robot.rightFront, robot.leftRear, robot.rightRear);

        /// IF THERE NEEDS TO BE MOVEMENT DURING INIT STAGE, UNCOMMENT
        //robot.initHasMovement();

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(
                new InstantCommand(() -> robot.intake.set(1))
        );
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
                new InstantCommand(() -> robot.intake.set(0))
        );

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(
                new InstantCommand(() -> robot.intake.set(-1))
        );
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
                new InstantCommand(() -> robot.intake.set(0))
        );

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

        telemetry.addData("Status", "Running");
        telemetry.addData("loop times", elapsedtime.milliseconds());
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
