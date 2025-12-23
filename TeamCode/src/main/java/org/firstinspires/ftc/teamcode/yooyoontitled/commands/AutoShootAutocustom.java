package org.firstinspires.ftc.teamcode.yooyoontitled.commands;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

public class AutoShootAutocustom extends ParallelCommandGroup {
    public AutoShootAutocustom(int speed) {
        Robot robot = Robot.getInstance();

        addCommands(
                new InstantCommand(() -> robot.shooter.shoot(speed)),
                new ConditionalCommand(
                        new InstantCommand(() -> robot.intake.start()
                        ),
                        new InstantCommand(() -> robot.intake.stop()),
                        () -> (robot.shooter1.getVelocity() > speed * 0.95)
                ).withTimeout(100)

        );
    }
}
