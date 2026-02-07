package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;
import static org.firstinspires.ftc.teamcode.yooyoontitled.ShootingUtils.*;
import static org.firstinspires.ftc.teamcode.yooyoontitled.sub.shooter.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

@Autonomous(name = "bluefarsecond", group = "auto")
public class bluefarsecond extends CommandOpMode{
    private final Robot robot = Robot.getInstance();
    private ElapsedTime timer;

    public static class Paths {
        public PathChain shootingpose;
        public PathChain pile1bump;
        public PathChain pileback;
        public PathChain pilepickup;
        public PathChain pileback2;
        public PathChain shoot;
        public PathChain secondpilealign;
        public PathChain secondpileget;
        public PathChain shoot2;
        public PathChain parking;

        public Paths(Follower follower) {
            shootingpose = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(64.160, 8.888),
                                    new Pose(62.370, 14.188)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(105))
                    .build();

            pile1bump = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(62.370, 14.188),
                                    new Pose(38.883, 11.340),
                                    new Pose(Robot.robotLength/2-0.1, 10.140)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(180))
                    .build();

            pileback = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(Robot.robotLength/2-0.1, 10.140),
                                    new Pose(30.888, Robot.robotWidth/2 + 0.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            pilepickup = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(45.888, Robot.robotWidth/2 +0.5),
                                    new Pose(43.888, Robot.robotWidth/2+0.1),
                                    new Pose(Robot.robotLength/2-0.1, Robot.robotWidth/2 +0.0)
                            )

                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
            pileback2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(Robot.robotLength/2-0.1, Robot.robotWidth/2 +0.5),
                                    new Pose(Robot.robotLength/2-0.5+10, 10)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();


            shoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(Robot.robotLength/2-0.5+10, 10),
                                    new Pose(17.370, 30.188),
                                    new Pose(55.370, 14.188)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(105))
                    .build();

            secondpilealign = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(55.370, 14.188),
                                    new Pose(55.536, 35.178),
                                    new Pose(45.930, 33)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(180))
                    .build();

            secondpileget = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.930, 39.794),
                                    new Pose(Robot.robotLength/2-0.5, 33)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(Robot.robotLength/2-0.5, 33),
                                    new Pose(55.370, 14.188)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(105))
                    .build();

            parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.370, 14.188),
                                    new Pose(51.732, 27.155)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }

    private Paths paths;

    /**
     * Turns the robot to face the goal
     */
    private void alignToGoal() {
        double targetHeading = calculateTargetHeading(robot.follower.getPose(), goals);
        robot.follower.turnToDegrees(Math.toDegrees(targetHeading));
    }

    public SequentialCommandGroup scorePreload() {
        return new SequentialCommandGroup(
                new WaitCommand(100),
                new InstantCommand(() -> robot.follower.setMaxPower(1)),
                new FollowPathCommand(robot.follower, paths.shootingpose, true),
                // Align to goal before shooting
                new InstantCommand(this::alignToGoal),
                new WaitCommand(500),
                new RepeatCommand(
                        new InstantCommand(() -> robot.shooter.shootAuto())
                ).withTimeout(6000),
                new InstantCommand(() -> robot.intake.stop()),
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_CLOSED))
        );
    }

    public SequentialCommandGroup grabPile1() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_CLOSED)),
                new InstantCommand(() -> robot.intake.start()),
                new InstantCommand(() -> robot.follower.setMaxPower(1)),
                new FollowPathCommand(robot.follower, paths.pile1bump, false).withTimeout(2000),
                new FollowPathCommand(robot.follower, paths.pileback, false).withTimeout(2000),
                new InstantCommand(() -> robot.intake.start()),
                new FollowPathCommand(robot.follower, paths.pilepickup, false).withTimeout(2000),
                new WaitCommand(500),
                new InstantCommand(() -> robot.follower.setMaxPower(1))
        );
    }

    public SequentialCommandGroup scorePile1() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, paths.shoot, true),
                // Align to goal before shooting
                new InstantCommand(this::alignToGoal),
                new InstantCommand(() -> robot.intake.stop()),
                new WaitCommand(500),
                new RepeatCommand(
                        new InstantCommand(() -> robot.shooter.shootAuto())
                ).withTimeout(3500),
                new InstantCommand(() -> robot.intake.stop()),
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_CLOSED))
        );
    }

    public SequentialCommandGroup grabpile2() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, paths.secondpilealign, true),
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_CLOSED)),
                new InstantCommand(() -> robot.intake.start()),
                new InstantCommand(() -> robot.follower.setMaxPower(1)),
                new FollowPathCommand(robot.follower, paths.secondpileget, false).withTimeout(2000),
                new WaitCommand(500),
                new InstantCommand(() -> robot.follower.setMaxPower(1))

        );
    }

    public SequentialCommandGroup scorePile2() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, paths.shoot2, true),
                // Align to goal before shooting
                new InstantCommand(this::alignToGoal),
                new InstantCommand(() -> robot.intake.stop()),
                new WaitCommand(500),
                new RepeatCommand(
                        new InstantCommand(() -> robot.shooter.shootAuto())
                ).withTimeout(3500),
                new InstantCommand(() -> robot.intake.stop()),
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_CLOSED))
        );
    }

    public SequentialCommandGroup park() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, paths.parking, true)
        );
    }

    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;
        goals = GoalColor.BLUE_GOAL;
        timer = new ElapsedTime();
        timer.reset();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);
        robot.stopperServo.set(STOPPER_CLOSED);

        // Initialize subsystems
        register(robot.intake, robot.shooter);

        // Generate paths
        paths = new Paths(robot.follower);
        robot.follower.setStartingPose(new Pose(64.160, 8.888, Math.toRadians(90)));
        robot.follower.setMaxPower(1);

        schedule(
                // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> robot.follower.update()),

                new SequentialCommandGroup(
                        new InstantCommand(),
                        scorePreload(),
                        grabpile2(),
                        scorePile2(),
                        grabPile1(),
                        scorePile1(),
                        park()


                )
        );
    }

    @Override
    public void initialize_loop(){
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();
        robot.shooter.shootsetspeed(900);
        telemetry.addData("timer", timer.milliseconds());
        telemetry.addData("followerIsBusy", robot.follower.isBusy());
        telemetry.addData("servo pos", robot.stopperServo.get());
        telemetry.addData("shooterReady", shooterReady);
        telemetry.addData("velocity motor", robot.shooter1.getVelocity());
        telemetry.addData("Heading", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("Heading Error", Math.toDegrees(robot.follower.getHeadingError()));

        telemetry.update();

        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        for(LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void end() {
        autoEndPose = robot.follower.getPose();
    }
}