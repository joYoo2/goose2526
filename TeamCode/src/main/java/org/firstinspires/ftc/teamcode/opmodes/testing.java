//package org.firstinspires.ftc.teamcode.opmodes;
//
//import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;
//
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.seattlesolvers.solverslib.command.CommandOpMode;
//import com.seattlesolvers.solverslib.command.InstantCommand;
//import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
//import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
//import com.seattlesolvers.solverslib.gamepad.GamepadEx;
//import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
//import com.seattlesolvers.solverslib.command.button.Trigger;
//import com.seattlesolvers.solverslib.util.InterpLUT;
//
//import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;
//
//@TeleOp(name = "Shooter Testing")
//public class testing extends CommandOpMode {
//    public GamepadEx driver, driver2;
//    public ElapsedTime gameTimer;
//    public static int shooterSpeed = 1000;
//    public static int adjustSpeed = 0;
//    public static double targetHeading;
//    public ElapsedTime elapsedtime;
//    private final Robot robot = Robot.getInstance();
//    private boolean shooterEnabled = false;
//
//    private static final double STOPPER_OPEN = 0.52;
//    private static final double STOPPER_CLOSED = 0;
//
//    // Thresholds for shooting (stopper opens earlier to give it time to move)
//    private static final double STOPPER_VELOCITY_THRESHOLD = 300; // Open stopper when within 100 RPM of target
//    private static final double INTAKE_VELOCITY_THRESHOLD = 50;   // Start intake when within 50 RPM of target
//    private static final double ALIGNMENT_THRESHOLD_DEG = 5.0;    // Alignment in degrees
//
//    private static final double ROBOT_WIDTH = 15.68;
//    private static final double ROBOT_LENGTH = 17.775591;
//
//
//    private static final Pose RED_GOAL = new Pose(154, 144, Math.toRadians(225));
//    private static final Pose BLUE_GOAL = new Pose(-10, 144, Math.toRadians(315));
//
//    private static final InterpLUT lookUpAutoShoot = new InterpLUT();
//
//    static {
//
//        lookUpAutoShoot.add(3.0, 750);
//        lookUpAutoShoot.add(4.0, 800);
//        lookUpAutoShoot.add(5.0, 850);
//        lookUpAutoShoot.add(6.0, 900);
//        lookUpAutoShoot.add(7.0, 950);
//        lookUpAutoShoot.add(8.0, 1000);
//        lookUpAutoShoot.add(9.0, 1050);
//        lookUpAutoShoot.add(10.0, 1100);
//        lookUpAutoShoot.add(11.0, 1150);
//        lookUpAutoShoot.add(10000.0, 1200);
//        lookUpAutoShoot.createLUT();
//    }
//
//    @Override
//    public void initialize(){
//        opModeType = OpModeType.TELEOP;
//
//        if (goals == null) {
//            goals = GoalColor.RED_GOAL;
//        }
//
//        // DO NOT REMOVE! Resetting FTCLib Command Scheduler
//        super.reset();
//
//        robot.init(hardwareMap);
//        elapsedtime = new ElapsedTime();
//        elapsedtime.reset();
//
//        register(robot.intake, robot.shooter);
//        driver = new GamepadEx(gamepad1);
//        driver2 = new GamepadEx(gamepad2);
//
//        robot.stopperServo.set(STOPPER_CLOSED);
//
//        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(
//                //new ParallelCommandGroup(
//                        new InstantCommand(() -> robot.intake.start())
//                //,)
//        );
//        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
//                new InstantCommand(() -> robot.intake.stop())
//        );
//
//        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(
//                new InstantCommand(() -> robot.intake.reverse())
//        );
//        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
//                new InstantCommand(() -> robot.intake.stop())
//        );
//
//
//        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(
//                //new ParallelCommandGroup(
//                new InstantCommand(() -> robot.intake.start())
//                //,)
//        );
//        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(
//                new InstantCommand(() -> robot.intake.stop())
//        );
//
//        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(
//                new InstantCommand(() -> robot.intake.reverse())
//        );
//        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(
//                new InstantCommand(() -> robot.intake.stop())
//        );
//
//
//        driver.getGamepadButton(GamepadKeys.Button.TOUCHPAD).whenPressed(
//                new InstantCommand(() -> {
//                    if(goals == GoalColor.BLUE_GOAL){
//                        gamepad1.rumble(1000);
//                        goals = GoalColor.RED_GOAL;
//                    }else{
//                        gamepad1.rumble(1000);
//                        goals = GoalColor.BLUE_GOAL;
//                    }
//                })
//        );
//
//        // DPAD_DOWN: Bottom corner relocalization (robot backed into corner)
//        // Position robot in bottom corner and press to set known position
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
//                new InstantCommand(() -> {
//                    if(goals == GoalColor.BLUE_GOAL){
//                        // Red bottom-right corner (x=144, y=0)
//                        robot.follower.setPose(new Pose(
//                                144 - ROBOT_WIDTH/2,
//                                0 + ROBOT_LENGTH/2,
//                                Math.toRadians(90)
//                        ));
//                        gamepad1.rumbleBlips(3);
//                    }else{
//                        // Blue bottom-left corner (x=0, y=0)
//                        robot.follower.setPose(new Pose(
//                                0 + ROBOT_WIDTH/2,
//                                0 + ROBOT_LENGTH/2,
//                                Math.toRadians(90)
//                        ));
//                        gamepad1.rumbleBlips(3);
//                    }
//                })
//        );
//
//
//
//
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
//                new InstantCommand(() -> adjustSpeed -= 20)
//        );
//
//        driver.getGamepadButton(GamepadKeys.Button.CIRCLE).whenPressed(
//                new InstantCommand(() -> robot.stopperServo.set(STOPPER_OPEN))
//        );
//        driver.getGamepadButton(GamepadKeys.Button.SQUARE).whenPressed(
//                new InstantCommand(() -> robot.stopperServo.set(STOPPER_CLOSED))
//        );
//
//        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
//                new InstantCommand(() -> adjustSpeed += 20)
//        );
//
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5).whenActive(
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> {
//                            // Auto-align to nearest corner
//                            double newHeading = calculateTargetHeading();
//                            targetHeading = newHeading;
//                            robot.follower.turnToDegrees(Math.toDegrees(newHeading));
//                            robot.follower.setConstraints(new PathConstraints(
//                                    0.995,
//                                    200,
//                                    1.5,
//                                    1
//                            ));
//                        })
//                )
//        );
//
//        new Trigger(() -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5).whenInactive(
//                new ParallelCommandGroup(
//                        new InstantCommand(() -> robot.shooter.stop()),
//                        new InstantCommand(() -> robot.intake.stop()),
//                        new InstantCommand(() -> robot.stopperServo.set(STOPPER_CLOSED)),
//                        new SequentialCommandGroup(
//                                new InstantCommand(() -> robot.follower.breakFollowing()),
//                                new InstantCommand(() -> robot.follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true)),
//                                new InstantCommand(() -> robot.follower.startTeleopDrive())
//                        )
//                )
//        );
//
//        driver.getGamepadButton(GamepadKeys.Button.TRIANGLE).whenPressed(
//                new InstantCommand(() -> {
//                    shooterEnabled = !shooterEnabled;
//                    if (!shooterEnabled) {
//                        robot.shooter.stop();
//                    }
//                    gamepad1.rumbleBlips(shooterEnabled ? 2 : 1);
//                })
//        );
//
//        super.run();
//    }
//
//    @Override
//    public void run() {
//        // Keep all the has movement init for until when TeleOp starts
//        if (gameTimer == null) {
//            robot.initHasMovement();
//            gameTimer = new ElapsedTime();
//        }
//
//        // DO NOT REMOVE! Runs FTCLib Command Scheduler
//        super.run();
//
//        targetHeading = calculateTargetHeading();
//        shooterSpeed = calculateShooterSpeed();
//
//        if(shooterEnabled){
//            robot.shooter.shoot(shooterSpeed + adjustSpeed);
//        }else{
//            robot.shooter.stop();
//        }
//
//       if(gamepad1.left_trigger > 0.5){
//            // Two-stage check: stopper gets lower threshold, intake gets stricter threshold
//            // Note: Alignment happens via auto-turn, but we don't wait for it to shoot
//            // (alignment uses drive motors which can cause voltage drop and slow shooter)
//            int targetSpeed = shooterSpeed + adjustSpeed;
//            double currentVelocity = robot.shooter1.getVelocity();
//            double angleError = Math.toDegrees(Math.abs(robot.follower.getPose().getHeading() - targetHeading));
//
//            // Stage 1: Open stopper early (gives servo time to move)
//            boolean stopperReady = currentVelocity > (targetSpeed - STOPPER_VELOCITY_THRESHOLD);
//
//            // Stage 2: Start intake only when velocity is ready (no alignment requirement)
//            boolean intakeReady = currentVelocity > (targetSpeed - INTAKE_VELOCITY_THRESHOLD);
//
//            telemetry.addData("DEBUG: Shooter Velocity", currentVelocity);
//            telemetry.addData("DEBUG: Target Speed", targetSpeed);
//            telemetry.addData("DEBUG: Stopper Ready? (within " + STOPPER_VELOCITY_THRESHOLD + ")", stopperReady);
//            telemetry.addData("DEBUG: Intake Ready? (within " + INTAKE_VELOCITY_THRESHOLD + ")", intakeReady);
//            telemetry.addData("DEBUG: Angle Error (deg)", angleError);
//
//            // Open stopper as soon as velocity is close (lower threshold)
//            if(stopperReady){
//                robot.stopperServo.set(STOPPER_OPEN);
//                telemetry.addData("DEBUG: Stopper", "OPEN (moving)");
//            } else {
//                robot.stopperServo.set(STOPPER_CLOSED);
//                telemetry.addData("DEBUG: Stopper", "CLOSED (waiting)");
//            }
//
//            // Start feeding when velocity is ready (driver controls alignment manually)
//            if(intakeReady){
//                robot.intake.start();
//                telemetry.addData("DEBUG: Intake", "FEEDING!");
//            } else {
//                robot.intake.stop();
//                telemetry.addData("DEBUG: Intake", "STOPPED (waiting for velocity)");
//            }
//
//            telemetry.addData("DEBUG: Stopper Actual Pos", robot.stopperServo.get());
//        } else {
//            // Trigger not pressed - just ensure stopper is closed
//            // Let the manual intake buttons (RIGHT/LEFT BUMPER) control the intake
//            robot.stopperServo.set(STOPPER_CLOSED);
//        }
//
//        robot.follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
//
//        if ((gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0) && robot.follower.isBusy()) {
//            robot.follower.breakFollowing();
//            robot.follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
//            robot.follower.startTeleopDrive();
//        }
//
//        telemetry.addData("=== ROBOT STATUS ===", "");
//        telemetry.addData("Alliance", goals);
//        telemetry.addData("Position (x, y)", "%.1f, %.1f", robot.follower.getPose().getX(), robot.follower.getPose().getY());
//        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(robot.follower.getPose().getHeading()));
//        telemetry.addData("", "");
//
//        telemetry.addData("=== SHOOTING DATA ===", "");
//        telemetry.addData("Target Corner", getTargetCornerName());
//        telemetry.addData("Distance (feet)", "%.2f", getDistanceToTarget());
//        telemetry.addData("Target Angle (deg)", "%.1f", Math.toDegrees(targetHeading));
//        telemetry.addData("Angle Error (deg)", "%.1f", Math.toDegrees(Math.abs(robot.follower.getPose().getHeading() - targetHeading)));
//        telemetry.addData("", "");
//
//        telemetry.addData("=== SHOOTER STATUS ===", "");
//        telemetry.addData("Shooter Enabled", shooterEnabled ? "ON" : "OFF");
//        telemetry.addData("Target Speed", shooterSpeed);
//        telemetry.addData("Adjust Speed", adjustSpeed);
//        telemetry.addData("Final Speed", shooterSpeed + adjustSpeed);
//        telemetry.addData("Actual Velocity", "%.0f", robot.shooter1.getVelocity());
//        telemetry.addData("Ready to Shoot", robot.shooter1.getVelocity() > shooterSpeed - 50);
//
//        telemetry.update();
//        robot.follower.update();
//
//        // Clear bulk cache for next loop
//        for(LynxModule hub : robot.allHubs) {
//            hub.clearBulkCache();
//        }
//        elapsedtime.reset();
//    }
//
//    @Override
//    public void end() {
//        autoEndPose = robot.follower.getPose();
//    }
//
//    /**
//     * Calculates the target heading to the nearest scoring corner
//     * based on current alliance color and robot position
//     */
//    private double calculateTargetHeading() {
//        Pose currentPose = robot.follower.getPose();
//        Pose targetCorner = getTargetCorner();
//
//        double dx = targetCorner.getX() - currentPose.getX();
//        double dy = targetCorner.getY() - currentPose.getY();
//
//        return Math.atan2(dy, dx);
//    }
//
//    /**
//     * Determines which goal to shoot at based on alliance color
//     * Goals are located in the TOP corners of the field only
//     * Red goal: top-right (144, 144)
//     * Blue goal: top-left (0, 144)
//     */
//    private Pose getTargetCorner() {
//        if(goals == GoalColor.RED_GOAL) {
//            return RED_GOAL;
//        } else {
//            return BLUE_GOAL;
//        }
//    }
//
//
//    private String getTargetCornerName() {
//        if(goals == GoalColor.RED_GOAL) {
//            return "Red Goal (Top-Right)";
//        } else {
//            return "Blue Goal (Top-Left)";
//        }
//    }
//
//
//    private double getDistanceToTarget() {
//        Pose currentPose = robot.follower.getPose();
//        Pose targetCorner = getTargetCorner();
//
//        double dx = (targetCorner.getX() - currentPose.getX()) / 12.0; // Convert to feet
//        double dy = (targetCorner.getY() - currentPose.getY()) / 12.0; // Convert to feet
//
//        return Math.sqrt(dx * dx + dy * dy);
//    }
//
//
//    private int calculateShooterSpeed() {
//        double distanceFeet = getDistanceToTarget();
//
//        // Define LUT bounds (should match the static block values)
//        final double MIN_DISTANCE = 3.0;
//        final double MAX_DISTANCE = 12.0;
//        final int MIN_SPEED = 800;
//        final int MAX_SPEED = 1700;
//
//        // Only use LUT if distance is within range
//        if (distanceFeet < MIN_DISTANCE) {
//            return MIN_SPEED;
//        } else if (distanceFeet > MAX_DISTANCE) {
//            return MAX_SPEED;
//        } else {
//            return (int) lookUpAutoShoot.get(distanceFeet);
//        }
//    }
//}