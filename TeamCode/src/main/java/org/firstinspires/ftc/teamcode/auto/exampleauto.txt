package org.firstinspires.ftc.teamcode.solverslib.opmode.Auto;

import static org.firstinspires.ftc.teamcode.solverslib.globals.Globals.*;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.solverslib.commandbase.commands.AutoShootInAutoFAR;
import org.firstinspires.ftc.teamcode.solverslib.globals.Robot;

@Autonomous(name = "FAR 9 BALL \uD83D\uDD35", group = "auto")
public class farAutoBlue extends CommandOpMode{
    private final Robot robot = Robot.getInstance();
    private ElapsedTime timer;

    //private final ArrayList<PathChain> paths = new ArrayList<>();

    // ALL PATHS
    private final Pose startPose = new Pose(88, 9, Math.toRadians(90)).mirror();
    private final Pose shootPose = new Pose(88, 15, Math.toRadians(63.5)).mirror();
    /// blue paths6
    private final Pose parkPose = new Pose(108,14, Math.toRadians(70)).mirror();
    private final Pose blueBottomPilePose = new Pose(50, 36, Math.toRadians(180));
    private final Pose blueBottomPileForwardPose = new Pose(13, 36, Math.toRadians(180));

    private final Pose blueDepotPilePose = new Pose(9, 33, Math.toRadians(250));
    private final Pose blueDepotPileForwardPose = new Pose(9, 9, Math.toRadians(250));
    private PathChain startToShoot, grabEndBlue, collectEndBlue, uncollectDepotBlue, shootEndBlue, grabDepotBlue, collectDepotBlue, shootDepotBlue, shootToPark;

    public void generatePath() {
        // PEDRO VISUALIZER: https://visualizer.pedropathing.com

        // Starting Pose (update this as well):
        //robot.follower.setStartingPose(startPose);

        //open .47
        //close .56


        startToShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        grabEndBlue = robot.follower.pathBuilder()
                .addPath(new BezierLine( shootPose, blueBottomPilePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), blueBottomPilePose.getHeading())
                .build();

        collectEndBlue = robot.follower.pathBuilder()
                .addPath(new BezierLine( blueBottomPilePose, blueBottomPileForwardPose))
                .setConstantHeadingInterpolation(blueBottomPileForwardPose.getHeading())
                .build();

        shootEndBlue = robot.follower.pathBuilder()
                .addPath(new BezierLine( blueBottomPileForwardPose, shootPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        grabDepotBlue = robot.follower.pathBuilder()
                .addPath(new BezierLine( shootPose, blueDepotPilePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), blueDepotPilePose.getHeading())
                .build();


        collectDepotBlue = robot.follower.pathBuilder()
                .addPath(new BezierLine(blueDepotPilePose, blueDepotPileForwardPose))
                .setLinearHeadingInterpolation(blueDepotPilePose.getHeading(), blueDepotPileForwardPose.getHeading())
                .build();

        uncollectDepotBlue = robot.follower.pathBuilder()
                .addPath(new BezierLine(blueDepotPileForwardPose, blueDepotPilePose))
                .setLinearHeadingInterpolation(blueDepotPilePose.getHeading(), blueDepotPileForwardPose.getHeading())
                .build();

        shootDepotBlue = robot.follower.pathBuilder()
                .addPath(new BezierLine(blueDepotPileForwardPose, blueDepotPilePose))
                .setLinearHeadingInterpolation(blueDepotPilePose.getHeading(), blueDepotPileForwardPose.getHeading())
                .addPath(new BezierLine(blueDepotPilePose, shootPose))
                .setLinearHeadingInterpolation(blueDepotPilePose.getHeading(), shootPose.getHeading())
                .build();


        shootToPark = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();

    }

    public SequentialCommandGroup startToShoot() {
        return new SequentialCommandGroup(
                //new WaitCommand(6000),
                new InstantCommand(() -> robot.outtake.shootAutoFar()),
                new FollowPathCommand(robot.follower, startToShoot, true),
                new InstantCommand(() -> robot.stopperServo.set(0.47)),
                new RepeatCommand(
                        new AutoShootInAutoFAR()
                ).withTimeout(3200),

                //new WaitCommand(3000),
                //new InstantCommand(() -> robot.outtake.stop()),
                new InstantCommand(() -> robot.intake.stop()),
                new InstantCommand(() -> robot.stopperServo.set(.56))

        );
    }


    public SequentialCommandGroup getEndBlue() {
        return new SequentialCommandGroup(
                //new WaitCommand(6000),
                new FollowPathCommand(robot.follower, grabEndBlue, false),
                new InstantCommand(() -> robot.follower.setMaxPower(.3)),
                //start intake
                new InstantCommand(() -> robot.intake.startCustom(1)),
                new FollowPathCommand(robot.follower, collectEndBlue, false).withTimeout(3000),
                //stop intake
                new InstantCommand(() ->robot.intake.stop()),
                new InstantCommand(() -> robot.follower.setMaxPower(1))

        );
    }

    public SequentialCommandGroup shootEndBlue() {
        return new SequentialCommandGroup(
                //new WaitCommand(6000),
                new FollowPathCommand(robot.follower, shootEndBlue, true),
                new InstantCommand(() -> robot.stopperServo.set(0.47)),
                new RepeatCommand(
                        new AutoShootInAutoFAR()
                ).withTimeout(2500),

                //new WaitCommand(3000),
                //new InstantCommand(() -> robot.outtake.stop()),
                new InstantCommand(() -> robot.intake.stop()),
                new InstantCommand(() -> robot.stopperServo.set(0.56)),
                new InstantCommand(() -> robot.follower.setMaxPower(.7))

        );
    }

    public SequentialCommandGroup getDepotBlue() {
        return new SequentialCommandGroup(
                //new WaitCommand(6000),
                new FollowPathCommand(robot.follower, grabDepotBlue, false),
                //start intake
                new InstantCommand(() -> robot.intake.startCustom(1)),
                new InstantCommand(() -> robot.follower.setMaxPower(.5)),
                new FollowPathCommand(robot.follower, collectDepotBlue, false).withTimeout(1000),
                new FollowPathCommand(robot.follower, uncollectDepotBlue, false).withTimeout(750),
                new FollowPathCommand(robot.follower, collectDepotBlue, false).withTimeout(750),
                new FollowPathCommand(robot.follower, uncollectDepotBlue, false).withTimeout(750),
                new FollowPathCommand(robot.follower, collectDepotBlue, false).withTimeout(750),
                new FollowPathCommand(robot.follower, uncollectDepotBlue, false).withTimeout(750),
                //stop intake
                new InstantCommand(() ->robot.intake.stop()),
                new InstantCommand(() -> robot.follower.setMaxPower(1))

        );
    }

    public SequentialCommandGroup shootDepotBlue() {
        return new SequentialCommandGroup(
                //new WaitCommand(6000),
                new FollowPathCommand(robot.follower, shootDepotBlue, true),
                new InstantCommand(() -> robot.stopperServo.set(0.47)),
                new RepeatCommand(
                        new AutoShootInAutoFAR()
                ).withTimeout(2500),

                //new WaitCommand(3000),
                new InstantCommand(() -> robot.outtake.stop()),
                new InstantCommand(() -> robot.intake.stop()),
                new InstantCommand(() -> robot.stopperServo.set(0.56))

        );
    }

    public SequentialCommandGroup shootToPark() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, shootToPark, true).withTimeout(3000)
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

        robot.stopperServo.set(0.56);


//        Limelight3A limelight;
//
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
//        limelight.start(); // This tells Limelight to start looking!
//
//        limelight.pipelineSwitch(1); // pipleline 1 is our AprilTags pipeline
//
//        LLResult result = limelight.getLatestResult();
//
//        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults(); // fiducials are special markers (like AprilTags)
//        for (LLResultTypes.FiducialResult fiducial : fiducials) {
//            int id = fiducial.getFiducialId(); // The ID number of the fiducial
//            if(id == 21){
//                randomizationMotif = RandomizationMotif.GREEN_LEFT;
//            }else if(id == 22){
//                randomizationMotif = RandomizationMotif.GREEN_MIDDLE;
//            }else if(id == 23){
//                randomizationMotif = RandomizationMotif.GREEN_RIGHT;
//            }else{
//                //failsafe
//                randomizationMotif = RandomizationMotif.GREEN_LEFT;
//            }
//        }
//
//

        // Initialize subsystems
        register(robot.intake, robot.outtake);

        //robot.initHasMovement();
        generatePath();
        robot.follower.setStartingPose(startPose);
        robot.follower.setMaxPower(1);

        schedule(
//                    // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> robot.follower.update()),
                new InstantCommand(() -> timer.reset()),

                new SequentialCommandGroup(
                        //new WaitCommand(27500),
                        new InstantCommand(),
                        startToShoot(),
                        getEndBlue(),
                        shootEndBlue(),
                        getDepotBlue(),
                        shootDepotBlue(),
                        shootToPark()
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

        robot.outtake.shootAutoFar();

        telemetry.addData("timer", timer.milliseconds());

        telemetry.addData("followerIsBusy", robot.follower.isBusy());
        telemetry.addData("repeat", test);
        telemetry.addData("shooterReady", shooterReady);
        telemetry.addData("servo pos", robot.stopperServo.get());
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
        autoEndPose = robot.follower.getPose();
    }
}

