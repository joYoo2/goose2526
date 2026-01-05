package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

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
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(115))
                    .build();

            pile1bump = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(62.370, 14.188),
                                    new Pose(38.883, 13.340),
                                    new Pose(8.688, 13.140)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                    .build();

            pileback = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(8.888, 13.140),
                                    new Pose(35.888, 9.840)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            pilepickup = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(35.888, 9.840),
                                    new Pose(8.888, 7.840)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            shoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(8.888, 7.840),
                                    new Pose(17.370, 30.188),
                                    new Pose(62.370, 14.188)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                    .build();

            secondpilealign = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(62.370, 14.188),
                                    new Pose(55.536, 35.178),
                                    new Pose(43.930, 35.794)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(180))
                    .build();

            secondpileget = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.930, 35.794),
                                    new Pose(0.016, 36.067)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            shoot2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(0.016, 36.067),
                                    new Pose(62.370, 14.188)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(115))
                    .build();

            parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(62.370, 14.188),
                                    new Pose(51.732, 27.155)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();
        }
    }

    private Paths paths;

    public SequentialCommandGroup scorePreload() {
        return new SequentialCommandGroup(
                new WaitCommand(100),
                new InstantCommand(() -> robot.follower.setMaxPower(1)),
                new FollowPathCommand(robot.follower, paths.shootingpose, true),
                new WaitCommand(500),
                new RepeatCommand(
                        new InstantCommand(() -> robot.shooter.shootAuto())
                ).withTimeout(5000),
                new InstantCommand(() -> robot.intake.stop()),
                new InstantCommand(() -> robot.stopperServo.set(0))
        );
    }

    public SequentialCommandGroup grabPile1() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.stopperServo.set(0)),
                new InstantCommand(() -> robot.intake.start()),
                new InstantCommand(() -> robot.follower.setMaxPower(1)),
                new FollowPathCommand(robot.follower, paths.pile1bump, false),
                new FollowPathCommand(robot.follower, paths.pileback, false),
                new FollowPathCommand(robot.follower, paths.pilepickup, false).withTimeout(2000),
                new WaitCommand(500),
                new InstantCommand(() -> robot.intake.stop()),
                new InstantCommand(() -> robot.follower.setMaxPower(1))
        );
    }

    public SequentialCommandGroup scorePile1() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, paths.shoot, true),
                new WaitCommand(500),
                new RepeatCommand(
                        new InstantCommand(() -> robot.shooter.shootAuto())
                ).withTimeout(5000),
                new InstantCommand(() -> robot.intake.stop()),
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.stopperServo.set(0))
        );
    }

    public SequentialCommandGroup grabpile2() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, paths.secondpilealign, true),
                new InstantCommand(() -> robot.stopperServo.set(0)),
                new InstantCommand(() -> robot.intake.start()),
                new InstantCommand(() -> robot.follower.setMaxPower(1)),
                new FollowPathCommand(robot.follower, paths.secondpileget, false).withTimeout(2000),
                new WaitCommand(500),
                new InstantCommand(() -> robot.intake.stop()),
                new InstantCommand(() -> robot.follower.setMaxPower(1))

        );
    }

    public SequentialCommandGroup scorePile2() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, paths.shoot2, true),
                new WaitCommand(500),
                new RepeatCommand(
                        new InstantCommand(() -> robot.shooter.shootAuto())
                ).withTimeout(5000),
                new InstantCommand(() -> robot.intake.stop()),
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.stopperServo.set(0))
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
        robot.stopperServo.set(0.52);

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