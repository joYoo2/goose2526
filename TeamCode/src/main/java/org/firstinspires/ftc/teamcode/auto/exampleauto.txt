package org.firstinspires.ftc.teamcode.solverslib.opmode.Auto;

import static org.firstinspires.ftc.teamcode.solverslib.globals.Globals.*;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.solverslib.commandbase.commands.AutoShootInAuto;
import org.firstinspires.ftc.teamcode.solverslib.commandbase.commands.AutoShootInAutoFAR;
import org.firstinspires.ftc.teamcode.solverslib.commandbase.commands.RapidShoot;
import org.firstinspires.ftc.teamcode.solverslib.globals.Robot;

@Autonomous(name = "15 BALL :O \uD83D\uDD35", group = "auto")
public class closeAutoBlue2 extends CommandOpMode{
    private final Robot robot = Robot.getInstance();
    private ElapsedTime timer;

    //private final ArrayList<PathChain> paths = new ArrayList<>();

    // ALL PATHS
//    private final Pose startPose = new Pose(16, 112, Math.toRadians(180));
    private final Pose startPose = new Pose(19, 116, Math.toRadians(180));

    private final Pose shootPose = new Pose(56, 84, Math.toRadians(180));//e
    private final Pose middlePose = new Pose(8.5, 60, Math.toRadians(180));
    private final Pose tapGate = new Pose(13, 61, Math.toRadians(145));

    private final Pose openGate = new Pose(9,58, Math.toRadians(105));
    private final Pose topPose = new Pose(12, 84, Math.toRadians(180));
    private final Pose bottomPose = new Pose(11, 36, Math.toRadians(180));
    private final Pose shootPose2 = new Pose(54, 104, Math.toRadians(180));
    private PathChain shootPreloads, getMiddle, shootMiddle, tapTheGate, openTheGate, shootGate, getTop, shootTop, getBottom, shootBottom;

    public void generatePath() {
        // PEDRO VISUALIZER: https://visualizer.pedropathing.com

        // Starting Pose (update this as well):
        //robot.follower.setStartingPose(startPose);

        shootPreloads = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        getMiddle = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(69, 50), middlePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), middlePose.getHeading())
                .build();

        shootMiddle = robot.follower.pathBuilder()
                .addPath(new BezierCurve(middlePose, new Pose(70,38), shootPose))
                .setLinearHeadingInterpolation(middlePose.getHeading(), shootPose.getHeading())
                .build();

        tapTheGate = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(72, 24), tapGate))
                .setLinearHeadingInterpolation(shootPose.getHeading(), tapGate.getHeading())
                .build();

        openTheGate = robot.follower.pathBuilder()
                .addPath(new BezierLine(tapGate, openGate))
                .setLinearHeadingInterpolation(tapGate.getHeading(), openGate.getHeading())
                .build();

        shootGate = robot.follower.pathBuilder()
                .addPath(new BezierCurve(openGate, new Pose(70,38), shootPose))
                .setLinearHeadingInterpolation(openGate.getHeading(), shootPose.getHeading())
                .build();

        getTop = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, topPose))
                .setConstantHeadingInterpolation(topPose.getHeading())
                .build();

        shootTop = robot.follower.pathBuilder()
                .addPath(new BezierLine(topPose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        getBottom = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(76.25, 18), bottomPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        shootBottom = robot.follower.pathBuilder()
                .addPath(new BezierLine(bottomPose, shootPose2))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();



    }

    public SequentialCommandGroup scorePreload() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> robot.intake.start()),
                new FollowPathCommand(robot.follower, shootPreloads, true),
                new InstantCommand(() -> robot.stopperServo.set(.5)),
                new RepeatCommand(
                        new RapidShoot()
                ).withTimeout(1000),
                new InstantCommand(() -> robot.stopperServo.set(.1))

        );
    }

    public SequentialCommandGroup grabTopBlue() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, getTop, true)

        );
    }

    public SequentialCommandGroup scoreTopBlue() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, shootTop, true),
                new InstantCommand(() -> robot.stopperServo.set(.5)),
                new RepeatCommand(
                        new RapidShoot()
                ).withTimeout(1000),
                new InstantCommand(() -> robot.stopperServo.set(.1))

        );
    }

    public SequentialCommandGroup grabMiddleBlue() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, getMiddle, true)

        );
    }

    public SequentialCommandGroup shootMiddleBlue() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, shootMiddle, true),
                new InstantCommand(() -> robot.stopperServo.set(.5)),
                new RepeatCommand(
                        new RapidShoot()
                ).withTimeout(1000),
                new InstantCommand(() -> robot.stopperServo.set(.1))

        );
    }

    public SequentialCommandGroup openGate() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, tapTheGate, true),
                new WaitCommand(500),
                new FollowPathCommand(robot.follower, openTheGate, true),
                new WaitCommand(1000)

        );
    }



    public SequentialCommandGroup scoreGate() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, shootGate, true),
                new InstantCommand(() -> robot.stopperServo.set(.5)),
                new RepeatCommand(
                        new RapidShoot()
                ).withTimeout(1000),
                new InstantCommand(() -> robot.stopperServo.set(.1))

        );
    }

    public SequentialCommandGroup grabBottomBlue() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, getBottom, true)

        );
    }

    public SequentialCommandGroup scoreBottomBlue() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, shootBottom, true),
                new InstantCommand(() -> robot.stopperServo.set(.5)),
                new RepeatCommand(
                        new RapidShoot()
                ).withTimeout(1000),
                new InstantCommand(() -> robot.stopperServo.set(.1))

        );
    }





    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;
        goalColor = GoalColor.BLUE_GOAL;
        timer = new ElapsedTime();
        timer.reset();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        robot.stopperServo.set(0.15);


//

        // Initialize subsystems
        register(robot.intake, robot.outtake);

        //robot.initHasMovement();
        generatePath();
        robot.follower.setStartingPose(startPose);
        robot.follower.setMaxPower(1);

        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        schedule(
//                    // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> robot.follower.update()),

                new SequentialCommandGroup(
                        new InstantCommand(),
                        scorePreload(),

                        grabMiddleBlue(),

                        shootMiddleBlue(),

                        openGate(),

                        scoreGate(),

                        //openGate(),

                        //scoreGate(),

                        grabTopBlue(),

                        scoreTopBlue(),

                        grabBottomBlue(),

                        scoreBottomBlue()
                )
        );



    }

    @Override
    public void initialize_loop(){

        //telemetry.addData("randomization:", randomizationMotif.toString());
        telemetry.update();
        //drawOnlyCurrent();
    }

    @Override
    public void run() {
        super.run();

        robot.outtake.moveTurret(47);

        robot.outtake.shootAuto();

        telemetry.addData("timer", timer.milliseconds());

        telemetry.addData("followerIsBusy", robot.follower.isBusy());
        telemetry.addData("repeat", test);
        telemetry.addData("servo pos", robot.stopperServo.get());
        telemetry.addData("shooterReady", shooterReady);
        telemetry.addData("velocity motor", robot.leftShooter.getVelocity());
        telemetry.addData("Heading", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("Heading Error", Math.toDegrees(robot.follower.getHeadingError()));


        telemetry.update(); // DO NOT REMOVE! Needed for telemetry

//        // Drawing path on panels?
        //draw();


        // DO NOT REMOVE! Removing this will return stale data since bulk caching is on Manual mode
        // Also only clearing the control hub to decrease loop times
        // This means if we start reading both hubs (which we aren't) we need to clear both
        for(LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void end() {
        turretEncoder = robot.turretMotor.getCurrentPosition();
        autoEndPose = robot.follower.getPose();
    }
}

