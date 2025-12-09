package org.firstinspires.ftc.teamcode.yooyoontitled.commands;


import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.shot;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

public class littleoutshootautofarrr extends ParallelCommandGroup {
    public littleoutshootautofarrr() {

        Robot robot = Robot.getInstance();
        addCommands(
                new InstantCommand(() -> robot.shooter.shoot(1700)),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.stopperServo.set(0.5)),
                                new WaitCommand(200),
                                new InstantCommand(() -> robot.intake.set(-0.5)),
                                new WaitCommand(50),
                                new InstantCommand(() -> robot.intake.set(1)),
                                new InstantCommand(() -> shot = true)
                        ),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.intake.set(-0.5)),
                                        new WaitCommand(50),
                                        new InstantCommand(() -> shot = false)

                                ), new SequentialCommandGroup(
                                    new InstantCommand(() -> robot.intake.set(0.1)),
                                    new InstantCommand(() -> robot.stopperServo.set(0))
                            ),
                                () -> (shot)
                        ).withTimeout(500),

                        () -> (robot.shooterMotor.getVelocity() > 1600)
                ).withTimeout(500)


        );
    }
}
