package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.GoalColor;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.OpModeType;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.STOPPER_CLOSED;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.autoEndPose;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.goalColor;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.opModeType;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.shooterReady;

import com.pedropathing.geometry.BezierCurve;
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
 * Blue Alliance Autonomous OpMode
 * 
 * Features:
 * - Turret auto-aiming before each shooting sequence
 * - Intake lift raises at gate, lowers when moving away
 * - Uses RapidShoot command for continuous ball feeding
 */
@Autonomous(name = "Blue close 15 gate", group = "Competition")
public class BlueAuto12gate extends CommandOpMode {
    private final Robot robot = Robot.getInstance();
    private ElapsedTime timer;

    // Auto-specific constants
    private static final double AUTO_HOOD_POS = 0.85;  // Hood position for auto shooting

    // Path poses
    private final Pose startPose = new Pose(19, 116, Math.toRadians(180));
    private final Pose shootPose = new Pose(50, 90, Math.toRadians(180));
    private final Pose middlePose = new Pose(8.5, 60, Math.toRadians(180));
    private final Pose tapGate = new Pose(11, 63.5, Math.toRadians(145));
    private final Pose topPose = new Pose(14, 84, Math.toRadians(180));
    private final Pose bottomPose = new Pose(12, 36, Math.toRadians(180));
    private final Pose shootPose2 = new Pose(56, 120, Math.toRadians(180));
    private final Pose park = new Pose(56, 120, Math.toRadians(180));


    // Path chains
    private PathChain shootPreloads, getMiddle, shootMiddle, tapTheGate, openTheGate;
    private PathChain shootGate, getTop, shootTop, getBottom, shootBottom, goPark;

    public void generatePaths() {
        shootPreloads = robot.follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        getMiddle = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(69, 50), middlePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), middlePose.getHeading())
                .build();

        shootMiddle = robot.follower.pathBuilder()
                .addPath(new BezierCurve(middlePose, new Pose(70, 38), shootPose))
                .setLinearHeadingInterpolation(middlePose.getHeading(), shootPose.getHeading())
                .build();

        tapTheGate = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(72, 24), tapGate))
                .setLinearHeadingInterpolation(shootPose.getHeading(), tapGate.getHeading())
                .build();



        shootGate = robot.follower.pathBuilder()
                .addPath(new BezierCurve(tapGate, new Pose(70, 38), shootPose))
                .setLinearHeadingInterpolation(tapGate.getHeading(), shootPose.getHeading())
                .build();

        getTop = robot.follower.pathBuilder()
                .addPath(new BezierLine(shootPose, topPose))
                .setConstantHeadingInterpolation(topPose.getHeading())
                .build();

        shootTop = robot.follower.pathBuilder()
                .addPath(new BezierLine(topPose, shootPose2))
                .setConstantHeadingInterpolation(shootPose2.getHeading())
                .build();

        getBottom = robot.follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(76.25, 18), bottomPose))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        shootBottom = robot.follower.pathBuilder()
                .addPath(new BezierLine(bottomPose, shootPose2))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();

        goPark = robot.follower.pathBuilder()
                .addPath(new BezierLine(bottomPose, park))
                .setConstantHeadingInterpolation(shootPose.getHeading())
                .build();
    }

    // ==================== COMMAND GROUPS ====================

    public SequentialCommandGroup scorePreload() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> robot.intake.lowerIntake()),

                new InstantCommand(() -> robot.intake.setState(Intake.IntakeState.INTAKING)),
                // Spin up shooter (calculate speed from shooting position)
                new InstantCommand(() -> {
                    robot.shooter.setTargetSpeed(robot.shooter.calculateTargetSpeedFromPose(shootPose, 0));
                    robot.shooter.spinUp();
                }),

                // Set hood angle based on destination distance
                new InstantCommand(() -> {
                    double hoodAngle = robot.shooter.calculateHoodAngleFromPose(shootPose, 0);
                    robot.hoodServo.set(hoodAngle);
                }),

                // Aim turret at future shooting position (not current position)
                new InstantCommand(() -> robot.turret.aimAtPose(shootPose)),

                // Drive to shooting position (shooter spinning up during travel)
                new FollowPathCommand(robot.follower, shootPreloads, true),
                
                // Enable shooting mode
                new InstantCommand(() -> Globe.shootingActive = true),
                
                // Wait for shooting (stopper controlled by shooterReady logic)
                new WaitCommand(1700),
                
                // Stop shooting, stop shooter, and disable auto-aim
                new InstantCommand(() -> Globe.shootingActive = false),
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.MANUAL))
        );
    }

    public SequentialCommandGroup grabMiddleBlue() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, getMiddle, true)
        );
    }

    public SequentialCommandGroup shootMiddleBlue() {
        return new SequentialCommandGroup(
                // Spin up shooter before driving (calculate speed from shooting position)
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
                new FollowPathCommand(robot.follower, shootMiddle, true),
                
                // Enable shooting mode
                new InstantCommand(() -> Globe.shootingActive = true),
                
                // Wait for shooting (stopper controlled by shooterReady logic)
                new WaitCommand(1500),
                
                // Stop shooting, stop shooter, and disable auto-aim
                new InstantCommand(() -> Globe.shootingActive = false),
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.MANUAL))
        );
    }

    public SequentialCommandGroup openGateSequence() {
        return new SequentialCommandGroup(
                // Switch to manual lift control and set gate height
                new InstantCommand(() -> robot.intake.setState(Intake.IntakeState.INTAKING)),
                 // 0.8
                
                // Tap the gate
                new FollowPathCommand(robot.follower, tapTheGate, true),
                new WaitCommand(1500),
                
                // Re-enable auto lift control and return to idle
                new InstantCommand(() -> {
                    robot.intake.setLiftMode(Intake.LiftMode.AUTO);
                    // periodic() will automatically lower intake via lowerIntake()
                })
        );
    }

    public SequentialCommandGroup scoreGate() {
        return new SequentialCommandGroup(
                // Spin up shooter before driving (calculate speed from shooting position)
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
                new FollowPathCommand(robot.follower, shootGate, true),
                
                // Enable shooting mode
                new InstantCommand(() -> Globe.shootingActive = true),
                
                // Wait for shooting (stopper controlled by shooterReady logic)
                new WaitCommand(1500),
                
                // Stop shooting, stop shooter, and disable auto-aim
                new InstantCommand(() -> Globe.shootingActive = false),
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.MANUAL))
        );
    }

    public SequentialCommandGroup grabTopBlue() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, getTop, true)
        );
    }

    public SequentialCommandGroup scoreTopBlue() {
        return new SequentialCommandGroup(
                // Spin up shooter before driving (calculate speed from shooting position)
                new InstantCommand(() -> {
                    robot.shooter.setTargetSpeed(robot.shooter.calculateTargetSpeedFromPose(shootPose2, 0));
                    robot.shooter.spinUp();
                }),
                
                // Set hood angle based on destination distance
                new InstantCommand(() -> {
                    double hoodAngle = robot.shooter.calculateHoodAngleFromPose(shootPose2, 0);
                    robot.hoodServo.set(hoodAngle);
                }),

                // Aim turret at future shooting position
                new InstantCommand(() -> robot.turret.aimAtPose(shootPose2)),
                
                // Drive to shooting position (shooter spinning up during travel)
                new FollowPathCommand(robot.follower, shootTop, true),
                
                // Enable shooting mode
                new InstantCommand(() -> Globe.shootingActive = true),
                
                // Wait for shooting (stopper controlled by shooterReady logic)
                new WaitCommand(1500),
                
                // Stop shooting, stop shooter, and disable auto-aim
                new InstantCommand(() -> Globe.shootingActive = false),
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.MANUAL))
        );
    }

    public SequentialCommandGroup grabBottomBlue() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, getBottom, true)
        );
    }

    public SequentialCommandGroup scoreBottomBlue() {
        return new SequentialCommandGroup(
                // Spin up shooter before driving (calculate speed from shooting position)
                new InstantCommand(() -> {
                    robot.shooter.setTargetSpeed(robot.shooter.calculateTargetSpeedFromPose(shootPose2, 0));
                    robot.shooter.spinUp();
                }),
                
                // Set hood angle based on destination distance
                new InstantCommand(() -> {
                    double hoodAngle = robot.shooter.calculateHoodAngleFromPose(shootPose2, 0);
                    robot.hoodServo.set(hoodAngle);
                }),

                // Aim turret at future shooting position
                new InstantCommand(() -> robot.turret.aimAtPose(shootPose2)),
                
                // Drive to shooting position (shooter spinning up during travel)
                new FollowPathCommand(robot.follower, shootBottom, true),
                
                // Enable shooting mode
                new InstantCommand(() -> Globe.shootingActive = true),
                
                // Wait for shooting (stopper controlled by shooterReady logic)
                new WaitCommand(1500),
                
                // Stop shooting, stop shooter, and disable auto-aim
                new InstantCommand(() -> Globe.shootingActive = false),
                new InstantCommand(() -> robot.shooter.stop()),
                new InstantCommand(() -> robot.turret.setMode(Turret.AimMode.MANUAL))
        );
    }

    public SequentialCommandGroup goAndPark() {
        return new SequentialCommandGroup(
                new FollowPathCommand(robot.follower, goPark, true)
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
        // Hood angle will be set in each shooting sequence based on distance
        
        // Safety reset: ensure intake lift mode is AUTO (in case of previous autonomous crash)
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

                // Main auto sequence
                new SequentialCommandGroup(
                        new InstantCommand(),  // Placeholder start
                        scorePreload(),
                        grabMiddleBlue(),
                        shootMiddleBlue(),
                        openGateSequence(),
                        scoreGate(),
                        openGateSequence(),
                        scoreGate(),
                        grabTopBlue(),
                        scoreTopBlue(),
                        goAndPark()
                )
        );
    }

    @Override
    public void initialize_loop() {
        telemetry.addData("Status", "Initialized - Blue Auto");
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
