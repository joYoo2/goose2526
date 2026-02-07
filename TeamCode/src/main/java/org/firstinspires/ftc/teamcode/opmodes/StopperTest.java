//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.yooyoontitled.Robot;
//
//@TeleOp(name = "Stopper Test")
//public class StopperTest extends LinearOpMode {
//
//    private final Robot robot = Robot.getInstance();
//    private double stopperPosition = 0.35;
//
//    private static final double STOPPER_OPEN = 0.725;
//    private static final double STOPPER_CLOSED = 0.24;
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
//
//        telemetry.addLine("Stopper Test OpMode");
//        telemetry.addLine("Controls:");
//        telemetry.addLine("  DPAD UP: OPEN");
//        telemetry.addLine("  DPAD DOWN: CLOSED");
//        telemetry.addLine("  DPAD LEFT: Position 0.2");
//        telemetry.addLine("  DPAD RIGHT: Position 0.65");
//        telemetry.addLine("  LEFT STICK Y: Fine control");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            // Preset positions with DPAD
//            if (gamepad1.dpad_up) {
//                stopperPosition = STOPPER_OPEN;
//            } else if (gamepad1.dpad_down) {
//                stopperPosition = STOPPER_CLOSED;
//            } else if (gamepad1.dpad_left) {
//                stopperPosition = 0.2;
//            } else if (gamepad1.dpad_right) {
//                stopperPosition = 0.65;
//            }
//
//            // Fine control with left stick
//            if (Math.abs(gamepad1.left_stick_y) > 0.1) {
//                stopperPosition -= gamepad1.left_stick_y * 0.01;
//                // Clamp between 0 and 1
//                stopperPosition = Math.max(0.0, Math.min(1.0, stopperPosition));
//            }
//
//            // Set the servo
//            robot.stopperServo.set(stopperPosition);
//
//            // Telemetry
//            telemetry.addData("=== STOPPER TEST ===", "");
//            telemetry.addData("Target Position", "%.3f", stopperPosition);
//            telemetry.addData("Actual Position", "%.3f", robot.stopperServo.get());
//            telemetry.addData("", "");
//            telemetry.addData("DPAD UP", "0.5 (typical open)");
//            telemetry.addData("DPAD DOWN", "0.35 (typical closed)");
//            telemetry.addData("DPAD LEFT", "0.2");
//            telemetry.addData("DPAD RIGHT", "0.65");
//            telemetry.addData("LEFT STICK Y", "Fine adjust");
//            telemetry.update();
//        }
//    }
//}