package org.firstinspires.ftc.teamcode.yooyoontitled.commands;


import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

public class littleoutshoot extends ParallelCommandGroup {
    public littleoutshoot() {

        Robot robot = Robot.getInstance();
        addCommands(
                new InstantCommand(() -> robot.shooter.shoot()),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(() -> robot.stopperServo.set(0.5)),
                                new WaitCommand(200),
                                new InstantCommand(() -> robot.intake.set(-0.5)),
                                new WaitCommand(75),
                                new InstantCommand(() -> robot.intake.set(1)),
                                new InstantCommand(() -> shot = true)
                        ),
                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(() -> robot.intake.set(-0.5)),
                                        new WaitCommand(200),
                                        new InstantCommand(() -> shot = false)

                                ), new SequentialCommandGroup(
                                    new InstantCommand(() -> robot.intake.set(0.1)),
                                    new InstantCommand(() -> robot.stopperServo.set(0))
                            ),
                                () -> (shot)
                        ).withTimeout(500),

                        () -> (robot.shooterMotor.getVelocity() > 1200)
                ).withTimeout(500)


        );
    }
}
