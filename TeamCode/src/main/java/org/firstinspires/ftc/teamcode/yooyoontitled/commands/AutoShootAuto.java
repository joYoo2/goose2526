package org.firstinspires.ftc.teamcode.yooyoontitled.commands;

import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

public class AutoShootAuto extends ParallelCommandGroup {
    private static final InterpLUT lookUpAutoShoot = new InterpLUT();

    static {
        lookUpAutoShoot.add(75.82, 1100);
        lookUpAutoShoot.createLUT();
    }

    public AutoShootAuto(double distance) {
        Robot robot = Robot.getInstance();
        int targetVelocity = (int) lookUpAutoShoot.get(distance);

        addCommands(
                new InstantCommand(() -> robot.shooter.shoot(targetVelocity)),
                new ConditionalCommand(
                        new InstantCommand(() -> robot.intake.start()
                        ),
                        new InstantCommand(() -> robot.intake.stop()),
                        () -> (robot.shooter1.getVelocity() > targetVelocity * 0.95)
                ).withTimeout(100)

        );
    }
}
