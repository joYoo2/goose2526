package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.GoalColor;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.OpModeType;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.STOPPER_CLOSED;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.autoEndPose;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.goalColor;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.opModeType;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.shooterReady;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Intake;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Turret;

/**
 * Blue Alliance Far Side Autonomous OpMode - 6 Ball + HPC (High Precision Cycling)
 * 
 * Sequence:
 * 1. Shoot preloads from starting position (5 second duration)
 * 2. Run 5 full cycles:
 *    - Drive to first intake position (collect samples)
 *    - Back up slightly
 *    - Drive to second intake position (collect more samples)
 *    - Return to shoot position and score (2 second duration)
 * 
 * Features:
 * - Turret auto-aiming at shootPose before each shooting sequence
 * - Hood angle auto-calculated from distance LUT
 * - Shooter pre-spinup during driving (for instant readiness)
 * - Intake state machine (INTAKING during collection, IDLE otherwise)
 * - Globe.shootingActive flag controls stopper automatically
 */
@Autonomous(name = "Blue Far 6+HPC", group = "Competition")
public class Bluefar6 extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private ElapsedTime timer;

    // Path poses
    private final Pose startPose = new Pose(41.5, 6.5, Math.toRadians(180));
    private final Pose shootPose = new Pose(47.5, 11, Math.toRadians(180));
    private final Pose firstIntake = new Pose(9.5, 9, Math.toRadians(180));
    private final Pose goBack = new Pose(24, 11, Math.toRadians(180));
    private final Pose secondIntake = new Pose(9.5, 11, Math.toRadians(180));

    // Path chains
    private PathChain firstShoot, goToIntake, backUp, goToIntake2, goToShoot;

    public void generatePaths() {
        firstShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goToIntake = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, firstIntake))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        backUp = robot.follower.pathBuilder()
                .addPath(new BezierLine(firstIntake, goBack))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goToIntake2 = robot.follower.pathBuilder()
                .addPath(new BezierLine(goBack, secondIntake))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        goToShoot = robot.follower.pathBuilder()
                .addPath(new BezierLine(secondIntake, shootPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    // ==================== COMMAND GROUPS ====================

    public SequentialCommandGroup shootPreloads() {
        return new SequentialCommandGroup(
                // Pre-spin shooter (calculate speed from shooting position)
                new InstantCommand(() -> {
                    robot.shooter.setTargetSpeed(robot.shooter.calculateTargetSpeedFromPose(shootPose, 0));
                    robot.shooter.spinUp();
                }),

                // Set hood angle based on destination distance
                new InstantCommand(() -> {
                    double hoodAngle = robot.shooter.calculateHoodAngleFromPose(shootPose, 0);
                    robot.hoodServo.set(hoodAngle);
                }),

                // Aim turret at future shooting position
                new InstantCommand(() -> robot.turret.aimAtPose(shootPose)),

                // Drive to shooting position (shooter spinning up during travel)
                new FollowPathCommand(robot.follower, firstShoot, false).withTimeout(3000),

                new WaitCommand(100),

                // Enable intake and shooting mode
                new InstantCommand(() -> robot.intake.setState(Intake.IntakeState.INTAKING)),
                new InstantCommand(() -> Globe.shootingActive = true),

                // Wait for shooting (stopper controlled by shooterReady logic)
                new WaitCommand(5000),  // Original duration: 5000ms

                // Stop shooting, stop intake, stop shooter, and disable auto-aim
                new InstantCommand(() -> Globe.shootingActive = false),
                new InstantCommand(() -> robot.intake.setState(Intake.IntakeState.IDLE)),
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.MANUAL))
        );
    }

    public SequentialCommandGroup fullCycle() {
        return new SequentialCommandGroup(
                // Start intake for sample collection
                new InstantCommand(() -> robot.intake.setState(Intake.IntakeState.INTAKING)),

                // Collection sequence: drive to intake positions (intake stays running throughout)
                new FollowPathCommand(robot.follower, goToIntake, false).withTimeout(3000),
                new FollowPathCommand(robot.follower, backUp, false).withTimeout(1000),
                new FollowPathCommand(robot.follower, goToIntake2, false).withTimeout(3000),

                // Pre-spin shooter before returning to shoot (calculate speed from shootPose)
                new InstantCommand(() -> {
                    robot.shooter.setTargetSpeed(robot.shooter.calculateTargetSpeedFromPose(shootPose, 0));
                    robot.shooter.spinUp();
                }),

                // Set hood angle based on destination distance
                new InstantCommand(() -> {
                    double hoodAngle = robot.shooter.calculateHoodAngleFromPose(shootPose, 0);
                    robot.hoodServo.set(hoodAngle);
                }),

                // Aim turret at future shooting position
                new InstantCommand(() -> robot.turret.aimAtPose(shootPose)),

                // Drive to shooting position (shooter spinning up during travel)
                new FollowPathCommand(robot.follower, goToShoot, false).withTimeout(3000),

                // Enable shooting mode
                new InstantCommand(() -> Globe.shootingActive = true),

                // Wait for shooting (stopper controlled by shooterReady logic)
                new WaitCommand(2000),  // Original duration: 2000ms

                // Stop shooting, stop intake, stop shooter, and disable auto-aim
                new InstantCommand(() -> Globe.shootingActive = false),
                new InstantCommand(() -> robot.intake.setState(Intake.IntakeState.IDLE)),
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.MANUAL))
        );
    }

    // ==================== OPMODE LIFECYCLE ====================

    @Override
    public void initialize() {
        opModeType = OpModeType.AUTO;
        goalColor = GoalColor.BLUE_GOAL;
        timer = new ElapsedTime();
        timer.reset();

        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
        super.reset();

        robot.init(hardwareMap);

        // Initial servo positions
        robot.stopperServo.set(STOPPER_CLOSED);

        // Safety reset: ensure intake lift mode is AUTO
        robot.intake.setLiftMode(Intake.LiftMode.AUTO);

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

                // Main auto sequence: shoot preloads + 5 full cycles
                new SequentialCommandGroup(
                        new InstantCommand(),  // Placeholder start
                        shootPreloads(),
                        fullCycle(),
                        fullCycle(),
                        fullCycle(),
                        fullCycle(),
                        fullCycle()
                )
        );
    }

    @Override
    public void initialize_loop() {
        telemetry.addData("Status", "Initialized - Blue Far 6+HPC");
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
