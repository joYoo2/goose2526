package org.firstinspires.ftc.teamcode.yooyoontitled;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.pedropathing.follower.Follower;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxModule.BulkCachingMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Drivetrain;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Intake;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Lights;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Shooter;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Turret;

import java.util.List;

public class Robot {

    // Drive motors
    public MotorEx leftFront, leftRear, rightFront, rightRear;

    // Shooter motors (shooterLeft is master — has encoder)
    public MotorEx shooterLeft, shooterRight;

    // Intake motors
    public MotorEx intakeLower, intakeUpper;

    // Servos
    public ServoEx hoodServo;
    public ServoEx intakeLiftLeft, intakeLiftRight;
    public CRServo turretLeft, turretRight;
    public AnalogInput turretEncoder;
    public Servo lightsServo;

    // Pedro Pathing follower
    public Follower follower;

    // Subsystems
    public Drivetrain drivetrain;
    public Shooter shooter;
    public Intake intake;
    public Lights lights;
    public Turret turret;

    // Bulk caching
    public List<LynxModule> allHubs;
    public LynxModule ControlHub;

    public static final double robotWidth  = 15.68;
    public static final double robotLength = 17.775591;

    private static Robot instance = new Robot();
    public boolean enabled;

    public static Robot getInstance() {
        if (instance == null) {
            instance = new Robot();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap) {
        // --- Drive motors ---
        leftFront  = new MotorEx(hardwareMap, "leftFront",  Motor.GoBILDA.RPM_435);
        leftRear   = new MotorEx(hardwareMap, "leftRear",   Motor.GoBILDA.RPM_435);
        rightFront = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        rightRear  = new MotorEx(hardwareMap, "rightRear",  Motor.GoBILDA.RPM_435);

        leftFront.setInverted(true);
        leftRear.setInverted(true);
        rightFront.setInverted(true);
        rightRear.setInverted(true);

        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // --- Shooter motors ---
        shooterLeft = new MotorEx(hardwareMap, "shooterLeft", Motor.GoBILDA.BARE);
        shooterLeft.setRunMode(Motor.RunMode.RawPower);
        shooterLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setInverted(true);

        shooterRight = new MotorEx(hardwareMap, "shooterRight", Motor.GoBILDA.BARE);
        shooterRight.setRunMode(Motor.RunMode.RawPower);
        shooterRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooterRight.setInverted(true);

        // --- Intake motors ---
        intakeLower = new MotorEx(hardwareMap, "intakeLower", Motor.GoBILDA.RPM_1150);
        intakeLower.setRunMode(Motor.RunMode.RawPower);

        intakeUpper = new MotorEx(hardwareMap, "intakeUpper", Motor.GoBILDA.RPM_1150);
        intakeUpper.setRunMode(Motor.RunMode.RawPower);
        intakeUpper.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        intakeUpper.setInverted(true);

        // --- Servos ---
        hoodServo       = new ServoEx(hardwareMap, "hood");
        intakeLiftLeft  = new ServoEx(hardwareMap, "intakeLiftLeft");
        intakeLiftRight = new ServoEx(hardwareMap, "intakeLiftRight");
        //lightsServo     = hardwareMap.get(Servo.class, "lights");

        // Safe initial positions
        hoodServo.set(Globe.HOOD_STATIC_POS);
        intakeLiftLeft.set(Globe.LIFT_RAISED);
        intakeLiftRight.set(Globe.LIFT_RAISED);

        turretLeft  = hardwareMap.get(CRServo.class, "turretLeft");
        turretRight = hardwareMap.get(CRServo.class, "turretRight");
        turretLeft.setPower(0);
        turretRight.setPower(0);
        turretEncoder = hardwareMap.get(AnalogInput.class, "turretEncoder");

        // --- Subsystems ---
        shooter    = new Shooter();
        intake     = new Intake();
        //lights     = new Lights();
        drivetrain = new Drivetrain();
        turret     = new Turret();

        // --- Follower ---
        follower = Constants.createFollower(hardwareMap);

        if (opModeType.equals(OpModeType.TELEOP)) {
            follower.setStartingPose(autoEndPose);
            follower.startTeleopDrive();
        }

        // --- Bulk caching (MANUAL mode for optimized loop times) ---
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(BulkCachingMode.MANUAL);
            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber())) {
                ControlHub = hub;
            }
        }
    }

    /** Called once when TeleOp actually starts (allows servo movement during init). */
    public void initHasMovement() { }
}
