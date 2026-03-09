//package org.firstinspires.ftc.teamcode.opmodes;
//
//import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;
//import static org.firstinspires.ftc.teamcode.yooyoontitled.ShootingUtils.*;
//import static org.firstinspires.ftc.teamcode.yooyoontitled.sub.Lights.lightsState;
//
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.gamepad.GamepadEx;
//import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
//import com.seattlesolvers.solverslib.command.button.Trigger;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Drawing;
//import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;
//import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Intake;
//import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Lights;
//
////@TeleOp(name = "Goose Teleop")
//public class twoplayeropmode extends CommandOpMode {
//
//    private final Robot robot = Robot.getInstance();
//    private GamepadEx driver, operator;
//    private ElapsedTime gameTimer;
//
//    private boolean shooterSpinning = false;
//    public static int adjustSpeed = 0;
//
//    private static final double ROBOT_WIDTH  = 15.68;
//    private static final double ROBOT_LENGTH = 17.775591;
//
//    @Override
//    public void initialize() {
//        opModeType = OpModeType.TELEOP;
//
//        if (goalColor == null) {
//            goalColor = GoalColor.RED_GOAL;
//        }
//
//        // DO NOT REMOVE — resets FTCLib command scheduler
//        super.reset();
//
//        robot.init(hardwareMap);
//
//        Drawing.init();
//
//        // Drivetrain is NOT registered here — follower.update() is called manually
//        // in run() to control exactly when it happens relative to drive commands.
//        register(robot.shooter, robot.intake, robot.lights);
//
//        lightsState = Lights.LightsState.TEAM_COLOR;
//
//        driver   = new GamepadEx(gamepad1);
//        operator = new GamepadEx(gamepad2);
//
//        // ── Gamepad 1 (Driver) ───────────────────────────────────────────────
//
//        // Circle: Toggle shooter spinning
//        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
//                new InstantCommand(() -> {
//                    shooterSpinning = !shooterSpinning;
//                    if (!shooterSpinning) robot.shooter.stop();
//                    gamepad1.rumbleBlips(shooterSpinning ? 2 : 1);
//                })
//        );
//
//        // Touchpad: Toggle goal color RED ↔ BLUE
//        driver.getGamepadButton(GamepadKeys.Button.TOUCHPAD).whenPressed(
//                new InstantCommand(() -> {
//                    goalColor = (goalColor == GoalColor.BLUE_GOAL) ? GoalColor.RED_GOAL : GoalColor.BLUE_GOAL;
//                    gamepad1.rumble(500);
//                })
//        );
//
//        // D-pad down: Relocalize to near corner
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
//                new InstantCommand(() -> {
//                    robot.follower.setPose(new Pose(
//                            goalColor == GoalColor.BLUE_GOAL
//                                    ? 141.5 - ROBOT_WIDTH / 2
//                                    : ROBOT_WIDTH / 2,
//                            ROBOT_LENGTH / 2,
//                            Math.toRadians(90)
//                    ));
//                    gamepad1.rumbleBlips(3);
//                })
//        );
//
//        // D-pad left: Relocalize to center
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
//                new InstantCommand(() -> {
//                    robot.follower.setPose(new Pose(
//                            goalColor == GoalColor.RED_GOAL
//                                    ? 141.5 / 2 + ROBOT_WIDTH / 2
//                                    : 141.5 / 2 - ROBOT_WIDTH / 2,
//                            ROBOT_LENGTH / 2,
//                            Math.toRadians(90)
//                    ));
//                    gamepad1.rumbleBlips(3);
//                })
//        );
//
//        // D-pad up: Relocalize to far wall
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                new InstantCommand(() -> {
//                    robot.follower.setPose(new Pose(
//                            goalColor == GoalColor.RED_GOAL
//                                    ? 141.5 / 2 + ROBOT_WIDTH / 2
//                                    : 141.5 / 2 - ROBOT_WIDTH / 2,
//                            141.5 - ROBOT_LENGTH / 2,
//                            Math.toRadians(90)
//                    ));
//                    gamepad1.rumbleBlips(3);
//                })
//        );
//
//        // Left trigger (hold): Auto-align to goal heading
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5).whenActive(
//                new InstantCommand(() -> {
//                    double heading = calculateTargetHeading(robot.follower.getPose(), goalColor);
//                    robot.follower.turnToDegrees(Math.toDegrees(heading));
//                    robot.follower.setConstraints(new PathConstraints(0.995, 200, 1.5, 1));
//                })
//        );
//
//        // Left trigger released: resume normal teleop drive
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5).whenInactive(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> robot.follower.breakFollowing()),
//                        new InstantCommand(() -> robot.follower.startTeleopDrive())
//                )
//        );
//
//        // ── Gamepad 2 (Operator) ─────────────────────────────────────────────
//
//        // Right bumper: Toggle intake INTAKING ↔ IDLE
//        operator.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
//                new InstantCommand(() -> {
//                    robot.intake.setState(
//                            robot.intake.getState() == Intake.IntakeState.INTAKING
//                                    ? Intake.IntakeState.IDLE
//                                    : Intake.IntakeState.INTAKING
//                    );
//                })
//        );
//
//        // Left bumper: Toggle intake REVERSED ↔ IDLE
//        operator.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new InstantCommand(() -> {
//                    robot.intake.setState(
//                            robot.intake.getState() == Intake.IntakeState.REVERSED
//                                    ? Intake.IntakeState.IDLE
//                                    : Intake.IntakeState.REVERSED
//                    );
//                })
//        );
//
//        // Right trigger (hold): Lower intake lift; release → raise
//        new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5).whenActive(
//                new InstantCommand(() -> robot.intake.lowerIntake())
//        );
//        new Trigger(() -> operator.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5).whenInactive(
//                new InstantCommand(() -> robot.intake.raiseIntake())
//        );
//
//        // D-pad up/down: Shooter speed fine-tune
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
//                new InstantCommand(() -> adjustSpeed += 20)
//        );
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
//                new InstantCommand(() -> adjustSpeed -= 20)
//        );
//
//        // D-pad left/right: Stopper fine adjust (enters manual mode)
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
//                new InstantCommand(() -> {
//                    robot.shooter.setStopperManual(robot.shooter.getStopperPosition() - 0.05);
//                    gamepad2.rumbleBlips(1);
//                })
//        );
//        operator.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
//                new InstantCommand(() -> {
//                    robot.shooter.setStopperManual(robot.shooter.getStopperPosition() + 0.05);
//                    gamepad2.rumbleBlips(1);
//                })
//        );
//
//        // Cross/X: Reset stopper to automatic mode
//        operator.getGamepadButton(GamepadKeys.Button.CROSS).whenPressed(
//                new InstantCommand(() -> {
//                    robot.shooter.setStopperAuto();
//                    gamepad2.rumbleBlips(2);
//                })
//        );
//
//        super.run();
//    }
//
//    @Override
//    public void run() {
//        // initHasMovement runs once when TeleOp loop actually starts
//        if (gameTimer == null) {
//            robot.initHasMovement();
//            gameTimer = new ElapsedTime();
//        }
//
//        // DO NOT REMOVE — runs FTCLib command scheduler (calls subsystem periodic())
//        super.run();
//
//        // Shooter: when spinning, calculate distance-based target speed and drive motors
//        if (shooterSpinning) {
//            double distanceFeet = getDistanceToTargetFeet(robot.follower.getPose(), goalColor);
//            double targetSpeed = robot.shooter.calculateTargetSpeed(distanceFeet) + adjustSpeed;
//            robot.shooter.setTargetSpeed(targetSpeed);
//            robot.shooter.spinUp();
//        }
//
//        // Drive (field-centric)
//        robot.follower.setTeleOpDrive(
//                -gamepad1.left_stick_y,
//                -gamepad1.left_stick_x,
//                -gamepad1.right_stick_x,
//                true
//        );
//
//        // If driver moves stick while follower is busy (e.g. auto-align), break out
//        if ((gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0)
//                && robot.follower.isBusy()) {
//            robot.follower.breakFollowing();
//            robot.follower.startTeleopDrive();
//        }
//
//        // Telemetry
//        telemetry.addData("Pos", "%.1f, %.1f",
//                robot.follower.getPose().getX(), robot.follower.getPose().getY());
//        telemetry.addData("Heading", "%.1f°",
//                Math.toDegrees(robot.follower.getPose().getHeading()));
//        telemetry.addData("Alliance", goalColor);
//        telemetry.addData("Shooter", shooterSpinning ? "ON" : "OFF");
//        telemetry.addData("Shooter Ready", shooterReady ? "YES" : "NO");
//        telemetry.addData("Adjust Speed", adjustSpeed);
//        telemetry.addData("Intake State", robot.intake.getState());
//        telemetry.update();
//
//        // Follower update + dashboard
//        robot.follower.update();
//        Drawing.drawRobot(robot.follower.getPose());
//        Drawing.drawPoseHistory(robot.follower.getPoseHistory());
//        Drawing.sendPacket();
//
//        // Clear bulk cache for next loop iteration
//        for (LynxModule hub : robot.allHubs) {
//            hub.clearBulkCache();
//        }
//    }
//
//    @Override
//    public void end() {
//        autoEndPose = robot.follower.getPose();
//    }
//}
