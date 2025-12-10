package org.firstinspires.ftc.teamcode.yooyoontitled;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.PoseTracker;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.hardware.SimpleServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.yooyoontitled.sub.intake;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.shooter;

import java.util.List;
public class Robot {
    public MotorEx leftFront, leftRear, rightRear, rightFront; //drivetrain wheels
    public MotorEx shooterL;
    public MotorEx shooterR;
    public Motor.Encoder shooterEncoder;

    public shooter shooter;

    public MotorEx intakeR;
    public MotorEx intakeL;

    public intake intake;

    public ServoEx rampServo, stopperServo;

    public Servo light;

    public Follower follower;
    public PoseTracker poseUpdater;

    /// the next two are for optimizing loop times
    public List<LynxModule> allHubs;
    public LynxModule ControlHub;


    private static Robot instance = new Robot();
    public boolean enabled;

    public static Robot getInstance() {
        if(instance == null){
            instance = new Robot();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap) {
        rightFront = new MotorEx(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        leftFront = new MotorEx(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        rightRear = new MotorEx(hardwareMap, "rightRear", Motor.GoBILDA.RPM_435);
        leftRear = new MotorEx(hardwareMap, "leftRear", Motor.GoBILDA.RPM_435);

        rightFront.setInverted(true);
        rightRear.setInverted(true);
        leftFront.setInverted(true);
        leftRear.setInverted(true);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);



        intakeR = new MotorEx(hardwareMap, "intakeR", Motor.GoBILDA.RPM_1150);
        intakeR.setRunMode(Motor.RunMode.RawPower);
        intakeR.setInverted(true);

        intakeL = new MotorEx(hardwareMap, "intakeL", Motor.GoBILDA.RPM_1150);
        intakeL.setRunMode(Motor.RunMode.RawPower);
        intakeL.setInverted(true);


        shooterL = new MotorEx(hardwareMap, "shooterL", Motor.GoBILDA.BARE);
        shooterL.setRunMode(Motor.RunMode.RawPower);
        shooterL.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooterL.setInverted(true);

        shooterR = new MotorEx(hardwareMap, "shooterR", Motor.GoBILDA.BARE);
        shooterR.setRunMode(Motor.RunMode.RawPower);
        shooterR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooterR.setInverted(true);

        shooterEncoder = new Motor(hardwareMap, "shooter").encoder;


        stopperServo = new ServoEx(hardwareMap, "stopper");
        rampServo = new ServoEx(hardwareMap, "angle");


        light = hardwareMap.get(Servo.class, "light");


        shooter = new org.firstinspires.ftc.teamcode.yooyoontitled.sub.shooter();

        //for optimizing loop times
        // Bulk reading enabled!
        // AUTO mode will bulk read by default and will redo and clear cache once the exact same read is done again
        // MANUAL mode will bulk read once per loop but needs to be manually cleared
        // Also in opModes only clear ControlHub cache as it is a hardware write
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            if (hub.isParent() && LynxConstants.isEmbeddedSerialNumber(hub.getSerialNumber())) {
                ControlHub = hub;
            }
        }


    }

    /// RUN WHATEVER IS IN THE INIT METHODS IN THE SUBSYSTEMS!!
    public void initHasMovement() {
        shooter.init();
        //kickServo.setPosition(0.5);
    }
}
