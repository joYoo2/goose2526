package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.APRIL_TAG_BLUE_GOAL;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.APRIL_TAG_RED_GOAL;
import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.LIMELIGHT_PIPELINE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Turret Calibration", group = "Test")
public class TurretCalibration extends LinearOpMode {

    @Override
    public void runOpMode() {
        CRServo turretLeft  = hardwareMap.get(CRServo.class, "turretLeft");
        CRServo turretRight = hardwareMap.get(CRServo.class, "turretRight");

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                turretLeft.setPower(-1);
                turretRight.setPower(-1);
            } else if (gamepad1.dpad_right) {
                turretLeft.setPower(1);
                turretRight.setPower(1);
            } else {
                turretLeft.setPower(0);
                turretRight.setPower(0);
            }

            // Poll Limelight
            boolean tagDetected = false;
            int detectedId = -1;
            double detectedTx = 0.0;
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                    int id = tag.getFiducialId();
                    if (id == APRIL_TAG_RED_GOAL || id == APRIL_TAG_BLUE_GOAL) {
                        tagDetected = true;
                        detectedId = id;
                        detectedTx = tag.getTargetXDegrees();
                        break;
                    }
                }
            }

            telemetry.addData("Tag detected", tagDetected ? ("YES id=" + detectedId) : "NO");
            telemetry.addData("tx (horiz err)", "%.2f deg", detectedTx);
            telemetry.addLine("DPAD L/R: drive turret");
            telemetry.update();
        }

        limelight.stop();
    }
}
