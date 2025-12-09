package org.firstinspires.ftc.teamcode.yooyoontitled.commands;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

public class AutoShoot extends ParallelCommandGroup {
    public AutoShoot() {
        Robot robot = Robot.getInstance();
        addCommands(
                new InstantCommand(() -> robot.shooter.shoot(1450)),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.stopperServo.set(0.1)),
//                                new WaitCommand(200),
                                new InstantCommand(() -> robot.intake.set(1)

                                )
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.intake.set(0.1)),
                                new InstantCommand(() -> robot.stopperServo.set(0.45))
                        ),
                        () -> (robot.shooterMotor.getVelocity() > 1200)
                ).withTimeout(500)

        );
    }
}
