package org.firstinspires.ftc.teamcode.yooyoontitled.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Intake;

/**
 * Command for rapid shooting in autonomous.
 * Sets up shooter and intake for continuous ball feeding.
 * 
 * Usage: Wrap with RepeatCommand and withTimeout() for timed shooting bursts.
 * Example: new RepeatCommand(new RapidShoot()).withTimeout(1000)
 */
public class RapidShoot extends ParallelCommandGroup {
    public RapidShoot() {
        Robot robot = Robot.getInstance();
        
        addCommands(
                // Enable shooting mode (triggers shooter ready logic)
                new InstantCommand(() -> Globe.shootingActive = true),
                
                // Set shooter to calculated target speed for current distance
                new InstantCommand(() -> robot.shooter.setTargetSpeed(
                        robot.shooter.calculateTargetSpeed(0))),
                
                // Spin up shooter
                new InstantCommand(() -> robot.shooter.spinUp()),
                
                // Start intake to feed balls
                new InstantCommand(() -> robot.intake.setState(Intake.IntakeState.INTAKING))
        );
    }
}
