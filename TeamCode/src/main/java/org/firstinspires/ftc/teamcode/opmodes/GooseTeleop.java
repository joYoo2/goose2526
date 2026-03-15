package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;

import org.firstinspires.ftc.teamcode.yooyoontitled.Globe;
import org.firstinspires.ftc.teamcode.yooyoontitled.ShootingUtils;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Intake;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Turret;

@TeleOp(name = "Goose Teleop")
public class GooseTeleop extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private GamepadEx driver, operator;
    private ElapsedTime gameTimer;

    private boolean shooterOn = false;
    public static int adjustSpeed = 0;
    private double hoodPos = HOOD_STATIC_POS;
    public static double hoodAdjust = 0.0;
    private static final double HOOD_STEP = 0.005;

    @Override
    public void initialize() {
        opModeType = OpModeType.TELEOP;

        if (goalColor == null) {
            goalColor = GoalColor.RED_GOAL;
        }


        // DO NOT REMOVE — resets FTCLib command scheduler
        super.reset();

        robot.init(hardwareMap);

        Drawing.init();

        // Drivetrain NOT registered — follower.update() called manually in run()
        register(robot.shooter, robot.intake, robot.turret);

        //lightsState = Lights.LightsState.TEAM_COLOR;

        driver   = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        // ── Gamepad 1 (Driver) ───────────────────────────────────────────────

        // Left trigger: Toggle turret auto-aim
        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5).whenActive(
                new InstantCommand(() -> {
                    robot.turret.toggleMode();
                    gamepad1.rumbleBlips(robot.turret.getMode() == Turret.AimMode.AUTO_AIM ? 2 : 1);
                })
        );

        // Triangle: Toggle lower intake spinning
        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> robot.intake.toggleLowerSpinning())
        );

        // Circle: Toggle shooter on/off entirely
        driver.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> {
                    shooterOn = !shooterOn;
                    shootingActive = false;
                    shooterSpinup = false;
                    if (!shooterOn) {
                        // When turning off, stop all shooting-related systems
                        robot.shooter.stop();
                    }
                    // When turning on, only enable shooterOn (don't auto-enable other systems)
                    gamepad1.rumbleBlips(shooterOn ? 2 : 1);
                })
        );

        // Cross: Toggle shooting active (full speed vs idle) - only works if shooter is on
        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> {
                    if (shooterOn) {
                        shootingActive = !shootingActive;
                        gamepad1.rumbleBlips(shootingActive ? 2 : 1);
                    } else {
                        shootingActive = false;
                    }
                })
        );

        // Touchpad: Toggle goal color RED ↔ BLUE
        driver.getGamepadButton(GamepadKeys.Button.TOUCHPAD).whenPressed(
                new InstantCommand(() -> {
                    goalColor = (goalColor == GoalColor.BLUE_GOAL) ? GoalColor.RED_GOAL : GoalColor.BLUE_GOAL;
                    gamepad1.rumble(500);
                })
        );

        // D-pad down: Relocalize to near corner
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> {
                    robot.follower.setPose(new Pose(
                            goalColor == GoalColor.BLUE_GOAL
                                    ? 141.5 - Robot.robotWidth / 2
                                    : Robot.robotWidth / 2,
                            Robot.robotLength / 2,
                            Math.toRadians(90)
                    ));
                    gamepad1.rumbleBlips(3);
                })
        );

        // D-pad left (hold): Turret left; release → stop
        new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_LEFT)).whenActive(
                new InstantCommand(() -> robot.turret.turnLeft())
        );
        new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_LEFT)).whenInactive(
                new InstantCommand(() -> robot.turret.stop())
        );

        // D-pad right (hold): Turret right; release → stop
        new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_RIGHT)).whenActive(
                new InstantCommand(() -> robot.turret.turnRight())
        );
        new Trigger(() -> driver.getButton(GamepadKeys.Button.DPAD_RIGHT)).whenInactive(
                new InstantCommand(() -> robot.turret.stop())
        );

        // D-pad up: Relocalize to far wall
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> {
                    robot.follower.setPose(new Pose(
                            goalColor == GoalColor.RED_GOAL
                                    ? 141.5 / 2 + Robot.robotWidth / 2
                                    : 141.5 / 2 - Robot.robotWidth / 2,
                            141.5 - Robot.robotLength / 2,
                            Math.toRadians(90)
                    ));
                    gamepad1.rumbleBlips(3);
                })
        );

        // Right bumper (hold): Intake; release → idle
        new Trigger(() -> driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)).whenActive(
                new InstantCommand(() -> robot.intake.setState(Intake.IntakeState.INTAKING))
        );
        new Trigger(() -> driver.getButton(GamepadKeys.Button.RIGHT_BUMPER)).whenInactive(
                new InstantCommand(() -> robot.intake.setState(Intake.IntakeState.IDLE))
        );

        // Left bumper (hold): Reverse intake; release → idle
        new Trigger(() -> driver.getButton(GamepadKeys.Button.LEFT_BUMPER)).whenActive(
                new InstantCommand(() -> robot.intake.setState(Intake.IntakeState.REVERSED))
        );
        new Trigger(() -> driver.getButton(GamepadKeys.Button.LEFT_BUMPER)).whenInactive(
                new InstantCommand(() -> robot.intake.setState(Intake.IntakeState.IDLE))
        );

        // Options: Reset turret zero position (re-center)
        driver.getGamepadButton(GamepadKeys.Button.SHARE).whenPressed(
                new InstantCommand(() -> {
                    robot.turret.resetZero();
                    gamepad1.rumbleBlips(1);
                })
        );

        // ── Gamepad 2 (Operator) ─────────────────────────────────────────────

        // Right bumper: Increase shooter speed adjustment
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> adjustSpeed += 25)
        );

        // Left bumper: Decrease shooter speed adjustment
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> adjustSpeed -= 25)
        );

        operator.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
                new InstantCommand(() -> {
                    shooterOn = !shooterOn;
                    if (!shooterOn) robot.shooter.stop();
                    gamepad2.rumbleBlips(shooterOn ? 2 : 1);
                })
        );

        // Cross: Toggle shooting active (full speed vs idle)
        operator.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> {
                    shootingActive = !shootingActive;
                    
                    gamepad2.rumbleBlips(shootingActive ? 2 : 1);
                })
        );

        // Triangle: Toggle spinup mode (full target speed without feeding - for warmup/testing) - only works if shooter is on
        operator.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
                new InstantCommand(() -> {
                    if (shooterOn) {
                        shooterSpinup = !shooterSpinup;
                        gamepad2.rumbleBlips(shooterSpinup ? 2 : 1);
                    } else {
                        shooterSpinup = false;
                    }
                })
        );

        // D-pad left (hold): Turret left; release → stop
        new Trigger(() -> operator.getButton(GamepadKeys.Button.DPAD_LEFT)).whenActive(
                new InstantCommand(() -> robot.turret.turnLeft())
        );
        new Trigger(() -> operator.getButton(GamepadKeys.Button.DPAD_LEFT)).whenInactive(
                new InstantCommand(() -> robot.turret.stop())
        );

        // D-pad right (hold): Turret right; release → stop
        new Trigger(() -> operator.getButton(GamepadKeys.Button.DPAD_RIGHT)).whenActive(
                new InstantCommand(() -> robot.turret.turnRight())
        );
        new Trigger(() -> operator.getButton(GamepadKeys.Button.DPAD_RIGHT)).whenInactive(
                new InstantCommand(() -> robot.turret.stop())
        );

        operator.getGamepadButton(GamepadKeys.Button.SHARE).whenPressed(
                new InstantCommand(() -> {
                    robot.turret.resetZero();
                    gamepad2.rumbleBlips(1);
                })
        );

        operator.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
                new InstantCommand(() -> robot.intake.toggleLowerSpinning())
        );

        // D-pad up/down: Hood servo fine-tune (handled in run())



        // ── Gamepad 2 (Operator, tuning) ─────────────────────────────────────────────

        new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5).whenActive(
                new InstantCommand(() -> {
                    robot.turret.toggleMode();
                    gamepad2.rumbleBlips(robot.turret.getMode() == Turret.AimMode.AUTO_AIM ? 2 : 1);
                })
        );

        operator.getGamepadButton(GamepadKeys.Button.TOUCHPAD).whenPressed(
                new InstantCommand(() -> {
                    robot.follower.setPose(new Pose(
                            goalColor == GoalColor.RED_GOAL
                                    ? 141.5 / 2 + Robot.robotWidth / 2
                                    : 141.5 / 2 - Robot.robotWidth / 2,
                            141.5 - Robot.robotLength / 2,
                            Math.toRadians(90)
                    ));
                    gamepad2.rumbleBlips(3);
                })
        );

        super.run();
    }

    @Override
    public void run() {
        // initHasMovement runs once when TeleOp loop actually starts
        if (gameTimer == null) {
            robot.initHasMovement();
            gameTimer = new ElapsedTime();
        }

        // Shooter: set state BEFORE periodic() runs so it has correct targetSpeed
        if (shooterOn) {
            double targetSpeed = robot.shooter.calculateTargetSpeed(adjustSpeed);
            if (shootingActive || shooterSpinup) {
                // Full target speed (shooterSpinup = warmup without feeding)
                robot.shooter.setTargetSpeed(targetSpeed);
            } else {
                robot.shooter.setTargetSpeed(targetSpeed * SHOOTER_IDLE_FRACTION);
            }
            robot.shooter.spinUp();
        }

        // DO NOT REMOVE — runs FTCLib command scheduler (calls subsystem periodic())
        super.run();

        // Hood servo: always auto-calculate from LUT + global adjust offset
        // D-pad up/down adjusts the global hood offset (like adjustSpeed for shooter)
        if (gamepad2.dpad_up)   hoodAdjust = Math.min(0.2, hoodAdjust + HOOD_STEP);
        if (gamepad2.dpad_down) hoodAdjust = Math.max(-0.2, hoodAdjust - HOOD_STEP);
        
        // Always auto-calculate hood angle from LUT based on distance to goal
        hoodPos = robot.shooter.calculateHoodAngle(hoodAdjust);
        robot.hoodServo.set(hoodPos);

        // Drive (field-centric)
        robot.follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        // Break auto-align if driver moves stick
        if ((gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0)
                && robot.follower.isBusy()) {
            robot.follower.breakFollowing();
            robot.follower.startTeleopDrive();
        }

        // Telemetry - compact LUT tuning view
        telemetry.addData("Pos", "%.1f, %.1f", robot.follower.getPose().getX(), robot.follower.getPose().getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("Alliance", goalColor);

        // Distance calculation
        double shooterDist = ShootingUtils.getDistanceToTargetFeet(robot.follower.getPose(), goalColor);
        telemetry.addData("Distance", "%.2f ft", shooterDist);

        // Shooter speed breakdown: LUT → offset → adjust → final
        double lutBase = robot.shooter.getLUTValue(shooterDist);
        double shooterTarget = robot.shooter.getTargetSpeed();
        double shooterCurrent = robot.shooter.getCurrentSpeed();
        telemetry.addData("Shooter Speed", "LUT: %.0f + Offset: %d + Adjust: %d = %.0f RPM",
                lutBase, robot.shooter.getOffset(), adjustSpeed, shooterTarget);
        telemetry.addData("Current Speed", "%.0f RPM (Error: %.0f)", shooterCurrent, shooterTarget - shooterCurrent);
        telemetry.addData("Shooter State", "%s | Ready: %s",
                shooterOn ? (shootingActive ? "ACTIVE" : "IDLE") : "OFF",
                shooterReady ? "YES" : "NO");

        // Hood angle breakdown: LUT → manual adjust
        double hoodLUT = robot.shooter.getHoodLUTValue(shooterDist);
        telemetry.addData("Hood Angle", "LUT: %.3f + Adjust: %.3f = %.3f", hoodLUT, hoodAdjust, hoodPos);

        // Other info
        telemetry.addData("Intake State", robot.intake.getState());
        telemetry.addData("Turret", "%s | Angle: %.1f° | Error: %.1f°",
                robot.turret.getMode(), robot.turret.getTurretDegrees(), robot.turret.lastError);
        telemetry.update();

        robot.follower.update();
        Drawing.drawRobot(robot.follower.getPose());
        Drawing.drawPoseHistory(robot.follower.getPoseHistory());
        Drawing.sendPacket();

        // Clear bulk cache for next loop
        for (LynxModule hub : robot.allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void end() {
        autoEndPose = robot.follower.getPose();
    }
}
