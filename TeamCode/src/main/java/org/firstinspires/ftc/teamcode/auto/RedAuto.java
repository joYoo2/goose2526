package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

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

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;
import org.firstinspires.ftc.teamcode.yooyoontitled.commands.RapidShoot;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Intake;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Turret;

/**
 * Red Alliance Autonomous OpMode
 * 
 * Features:
 * - Turret auto-aiming before each shooting sequence
 * - Intake lift raises at gate, lowers when moving away
 * - Uses RapidShoot command for continuous ball feeding
 * - Mirrored from BlueAuto for Red alliance side
 */
@Autonomous(name = "Red Auto", group = "Competition")
public class RedAuto extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private ElapsedTime timer;

    // Auto-specific constants
    private static final double AUTO_HOOD_POS = 0.5;  // Hood position for auto shooting

    // Path poses (mirrored for Red alliance - X = 141.5 - blue_x)
    private final Pose startPose = new Pose(122.5, 116, Math.toRadians(0));
    private final Pose shootPose = new Pose(85.5, 84, Math.toRadians(0));
    private final Pose middlePose = new Pose(133, 60, Math.toRadians(0));
    private final Pose tapGate = new Pose(128.5, 61, Math.toRadians(215));
    private final Pose openGate = new Pose(132.5, 58, Math.toRadians(255));
    private final Pose topPose = new Pose(129.5, 84, Math.toRadians(0));
    private final Pose bottomPose = new Pose(130.5, 36, Math.toRadians(0));
    private final Pose shootPose2 = new Pose(87.5, 104, Math.toRadians(0));

    // Path chains
    private PathChain shootPreloads, getMiddle, shootMiddle, tapTheGate, openTheGate;
    private PathChain shootGate, getTop, shootTop, getBottom, shootBottom;

    public void generatePaths() {
        shootPreloads = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        getMiddle = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(72.5, 50), middlePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), middlePose.getHeading())
                .build();

        shootMiddle = robot.follower.pathBuilder()
                .addPath(new BezierCurve(middlePose, new Pose(71.5, 38), shootPose))
                .setLinearHeadingInterpolation(middlePose.getHeading(), shootPose.getHeading())
                .build();

        tapTheGate = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(69.5, 24), tapGate))
                .setLinearHeadingInterpolation(shootPose.getHeading(), tapGate.getHeading())
                .build();

        openTheGate = robot.follower.pathBuilder()
                .addPath(new BezierLine(tapGate, openGate))
                .setLinearHeadingInterpolation(tapGate.getHeading(), openGate.getHeading())
                .build();

        shootGate = robot.follower.pathBuilder()
                .addPath(new BezierCurve(openGate, new Pose(71.5, 38), shootPose))
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
                .addPath(new BezierCurve(shootPose, new Pose(65.25, 18), bottomPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        shootBottom = robot.follower.pathBuilder()
                .addPath(new BezierLine(bottomPose, shootPose2))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();
    }

    // ==================== COMMAND GROUPS ====================

    public SequentialCommandGroup scorePreload() {
        return new SequentialCommandGroup(
                // Start intake and spin up shooter (calculate speed from shooting position)
                new InstantCommand(() -> robot.intake.setState(Intake.IntakeState.INTAKING)),
                new InstantCommand(() -> {
                    robot.shooter.setTargetSpeed(robot.shooter.calculateTargetSpeedFromPose(shootPose, 0));
                    robot.shooter.spinUp();
                }),
                
                // Drive to shooting position (shooter spinning up during travel)
                new FollowPathCommand(robot.follower, shootPreloads, true),
                
                // Enable turret auto-aim
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.AUTO_AIM)),
                
                // Open stopper and shoot
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_OPEN)),
                new RepeatCommand(new RapidShoot()).withTimeout(1000),
                
                // Close stopper, stop shooter, and disable auto-aim
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_CLOSED)),
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.MANUAL))
        );
    }

    public SequentialCommandGroup grabMiddleRed() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, getMiddle, true)
        );
    }

    public SequentialCommandGroup shootMiddleRed() {
        return new SequentialCommandGroup(
                // Spin up shooter before driving (calculate speed from shooting position)
                new InstantCommand(() -> {
                    robot.shooter.setTargetSpeed(robot.shooter.calculateTargetSpeedFromPose(shootPose, 0));
                    robot.shooter.spinUp();
                }),
                
                // Drive to shooting position (shooter spinning up during travel)
                new FollowPathCommand(robot.follower, shootMiddle, true),
                
                // Enable turret auto-aim
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.AUTO_AIM)),
                
                // Open stopper and shoot
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_OPEN)),
                new RepeatCommand(new RapidShoot()).withTimeout(1000),
                
                // Close stopper, stop shooter, and disable auto-aim
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_CLOSED)),
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.MANUAL))
        );
    }

    public SequentialCommandGroup openGateSequence() {
        return new SequentialCommandGroup(
                // Raise intake to gate height before approaching
                new InstantCommand(() -> robot.intake.setGateHeight()),
                
                // Tap the gate
                new FollowPathCommand(robot.follower, tapTheGate, true),
                new WaitCommand(500),
                
                // Open the gate
                new FollowPathCommand(robot.follower, openTheGate, true),
                new WaitCommand(1000),
                
                // Lower intake back to normal height
                new InstantCommand(() -> robot.intake.lowerIntake())
        );
    }

    public SequentialCommandGroup scoreGate() {
        return new SequentialCommandGroup(
                // Spin up shooter before driving (calculate speed from shooting position)
                new InstantCommand(() -> {
                    robot.shooter.setTargetSpeed(robot.shooter.calculateTargetSpeedFromPose(shootPose, 0));
                    robot.shooter.spinUp();
                }),
                
                // Drive to shooting position (shooter spinning up during travel)
                new FollowPathCommand(robot.follower, shootGate, true),
                
                // Enable turret auto-aim
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.AUTO_AIM)),
                
                // Open stopper and shoot
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_OPEN)),
                new RepeatCommand(new RapidShoot()).withTimeout(1000),
                
                // Close stopper, stop shooter, and disable auto-aim
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_CLOSED)),
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.MANUAL))
        );
    }

    public SequentialCommandGroup grabTopRed() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, getTop, true)
        );
    }

    public SequentialCommandGroup scoreTopRed() {
        return new SequentialCommandGroup(
                // Spin up shooter before driving (calculate speed from shooting position)
                new InstantCommand(() -> {
                    robot.shooter.setTargetSpeed(robot.shooter.calculateTargetSpeedFromPose(shootPose, 0));
                    robot.shooter.spinUp();
                }),
                
                // Drive to shooting position (shooter spinning up during travel)
                new FollowPathCommand(robot.follower, shootTop, true),
                
                // Enable turret auto-aim
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.AUTO_AIM)),
                
                // Open stopper and shoot
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_OPEN)),
                new RepeatCommand(new RapidShoot()).withTimeout(1000),
                
                // Close stopper, stop shooter, and disable auto-aim
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_CLOSED)),
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.MANUAL))
        );
    }

    public SequentialCommandGroup grabBottomRed() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, getBottom, true)
        );
    }

    public SequentialCommandGroup scoreBottomRed() {
        return new SequentialCommandGroup(
                // Spin up shooter before driving (calculate speed from shooting position)
                new InstantCommand(() -> {
                    robot.shooter.setTargetSpeed(robot.shooter.calculateTargetSpeedFromPose(shootPose2, 0));
                    robot.shooter.spinUp();
                }),
                
                // Drive to shooting position (shooter spinning up during travel)
                new FollowPathCommand(robot.follower, shootBottom, true),
                
                // Enable turret auto-aim
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.AUTO_AIM)),
                
                // Open stopper and shoot
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_OPEN)),
                new RepeatCommand(new RapidShoot()).withTimeout(1000),
                
                // Close stopper, stop shooter, and disable auto-aim
                new InstantCommand(() -> robot.stopperServo.set(STOPPER_CLOSED)),
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.MANUAL))
        );
    }

    // ==================== OPMODE LIFECYCLE ====================

    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;
        goalColor = GoalColor.RED_GOAL;
        timer = new ElapsedTime();
        timer.reset();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initial servo positions
        robot.stopperServo.set(STOPPER_CLOSED);
        robot.hoodServo.set(AUTO_HOOD_POS);

        // Register subsystems
        register(robot.shooter, robot.intake, robot.turret);

        // Generate paths
        generatePaths();
        robot.follower.setStartingPose(startPose);
        robot.follower.setMaxPower(1);

        // Schedule commands
        schedule(
                // DO NOT REMOVE: updates follower to follow path
                new RunCommand(() -> robot.follower.update()),

                // Main auto sequence
                new SequentialCommandGroup(
                        new InstantCommand(),  // Placeholder start
                        scorePreload(),
                        grabMiddleRed(),
                        shootMiddleRed(),
                        openGateSequence(),
                        scoreGate(),
                        grabTopRed(),
                        scoreTopRed(),
                        grabBottomRed(),
                        scoreBottomRed()
                )
        );
    }

    @Override
    public void initialize_loop() {
        telemetry.addData("Status", "Initialized - Red Auto");
        telemetry.addData("Goal Color", goalColor);
        telemetry.update();
    }

    @Override
    public void run() {
        super.run();

        // Telemetry
        telemetry.addData("Timer", "%.1f s", timer.seconds());
        telemetry.addData("Follower Busy", robot.follower.isBusy());
        telemetry.addData("Turret Mode", robot.turret.getMode());
        telemetry.addData("Turret Angle", "%.1f deg", robot.turret.getTurretDegrees());
        telemetry.addData("Aim Error", "%.1f deg", robot.turret.lastError);
        telemetry.addData("Shooter Ready", shooterReady);
        telemetry.addData("Shooter RPM", "%.0f", robot.shooter.getCurrentSpeed());
        telemetry.addData("Stopper Pos", "%.2f", robot.stopperServo.get());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.update();

        // Clear bulk cache
        for (LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void end() {
        // Save ending pose for teleop handoff
        autoEndPose = robot.follower.getPose();
        
        // Clean up
        Globe.shootingActive = false;
        robot.turret.setMode(Turret.AimMode.MANUAL);
    }
}
