package org.firstinspires.ftc.teamcode.yooyoontitled;

import static org.firstinspires.ftc.teamcode.yooyoontitled.Globe.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.Lights;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.intake;
import org.firstinspires.ftc.teamcode.yooyoontitled.sub.shooter;

import java.util.List;
public class Robot {
    //eventually remove
    private final Pose autoEndPose = new Pose(0.5*robotWidth, 0.5*robotLength, Math.toRadians(90)); //e

    public MotorEx leftFront, leftRear, rightRear, rightFront; //drivetrain wheels
    public MotorEx shooter1;
    public MotorEx shooter2;
    public Motor.Encoder shooterEncoder;

    public shooter shooter;

    public MotorEx intakeR;
    public MotorEx intakeL;

    public Servo lightning;

    public intake intake;

    public ServoEx rampServo, stopperServo;

    public Lights lights;
    public static double robotLength = 17.775591;
    public static double robotWidth = 15.68;

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

        intakeL = new MotorEx(hardwareMap, "intakeL", Motor.GoBILDA.RPM_1150);
        intakeL.setRunMode(Motor.RunMode.RawPower);
        intakeL.setInverted(true);

        shooter1 = new MotorEx(hardwareMap, "shooter1", Motor.GoBILDA.BARE);
        shooter1.setRunMode(Motor.RunMode.RawPower);
        shooter1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        shooter2 = new MotorEx(hardwareMap, "shooter2", Motor.GoBILDA.BARE);
        shooter2.setRunMode(Motor.RunMode.RawPower);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        shooterEncoder = new Motor(hardwareMap, "shooter1").encoder;


        stopperServo = new ServoEx(hardwareMap, "stopper");


        lightning = hardwareMap.get(Servo.class, "light");

        lights = new org.firstinspires.ftc.teamcode.yooyoontitled.sub.Lights();

        intake = new org.firstinspires.ftc.teamcode.yooyoontitled.sub.intake();
        shooter = new org.firstinspires.ftc.teamcode.yooyoontitled.sub.shooter();



        follower = Constants.createFollower(hardwareMap);

        if(opModeType.equals(OpModeType.TELEOP)) {
            follower.setStartingPose(autoEndPose);
            follower.startTeleopDrive();

        } else{
            //follower.setStartingPose(new Pose(0, 0, 0));
        }

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
