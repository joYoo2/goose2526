package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(0)                                                                    // TODO: re-tune — robot mass in kg
            .holdPointHeadingScaling(1)
            .forwardZeroPowerAcceleration(0)                                            // TODO: re-tune — run forward decel test
            .lateralZeroPowerAcceleration(0)                                            // TODO: re-tune — run lateral decel test
            .translationalPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0))           // TODO: re-tune translational PIDF
            .headingPIDFCoefficients(new PIDFCoefficients(0, 0, 0, 0))                 // TODO: re-tune heading PIDF
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0, 0, 0, 0, 0))        // TODO: re-tune drive PIDF
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.5, 50, 0.5, 0.5); // TODO: re-tune path constraints

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)                                                             // TODO: measure forward pod Y offset (inches)
            .strafePodX(0)                                                              // TODO: measure strafe pod X offset (inches)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(0)                                                               // TODO: re-tune max forward velocity (in/s)
            .yVelocity(0)                                                               // TODO: re-tune max lateral velocity (in/s)
            .leftFrontMotorName("leftFront")
            .leftRearMotorName("leftRear")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
