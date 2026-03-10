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
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Intake;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Turret;

@TeleOp(name = "Goose Teleop")
public class GooseTeleop extends CommandOpMode {

    private final Robot robot = Robot.getInstance();
    private GamepadEx driver, operator;
    private ElapsedTime gameTimer;

    private boolean shooterSpinning = false;
    public static int adjustSpeed = 0;

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

        // Circle: Toggle shooter spinning
        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
                new InstantCommand(() -> {
                    shooterSpinning = !shooterSpinning;
                    if (!shooterSpinning) robot.shooter.stop();
                    gamepad1.rumbleBlips(shooterSpinning ? 2 : 1);
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

        // ── Gamepad 2 (Operator) ─────────────────────────────────────────────

        // Right bumper: Toggle intake INTAKING ↔ IDLE
        /*
        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.intake.setState(
                        robot.intake.getState() == Intake.IntakeState.INTAKING
                                ? Intake.IntakeState.IDLE
                                : Intake.IntakeState.INTAKING
                ))
        );

        // Left bumper: Toggle intake REVERSED ↔ IDLE
        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> robot.intake.setState(
                        robot.intake.getState() == Intake.IntakeState.REVERSED
                                ? Intake.IntakeState.IDLE
                                : Intake.IntakeState.REVERSED
                ))
        );
        */

        // D-pad up/down: Shooter speed fine-tune
        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> adjustSpeed += 20)
        );
        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> adjustSpeed -= 20)
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

        // DO NOT REMOVE — runs FTCLib command scheduler (calls subsystem periodic())
        super.run();

        // Shooter: distance-based target speed, drive motors when spinning
        if (shooterSpinning) {
            double targetSpeed = robot.shooter.calculateTargetSpeed() + adjustSpeed;
            robot.shooter.setTargetSpeed(targetSpeed);
            robot.shooter.spinUp();
        }


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

        // Telemetry
        telemetry.addData("Pos", "%.1f, %.1f",
                robot.follower.getPose().getX(), robot.follower.getPose().getY());
        telemetry.addData("Heading", "%.1f deg",
                Math.toDegrees(robot.follower.getPose().getHeading()));
        telemetry.addData("Alliance", goalColor);
        telemetry.addData("Shooter", shooterSpinning ? "ON" : "OFF");
        telemetry.addData("Shooter Ready", shooterReady ? "YES" : "NO");
        telemetry.addData("Adjust Speed", adjustSpeed);
        telemetry.addData("Intake State", robot.intake.getState());
        telemetry.addData("Turret Mode", robot.turret.getMode());
        telemetry.addData("Turret Angle", "%.1f deg", robot.turret.getTurretDegrees());
        telemetry.addData("Aim Error", "%.1f deg", robot.turret.lastError);
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
