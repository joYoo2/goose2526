package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.7913)                                                                    // TODO: re-tune — robot mass in kg
            .holdPointHeadingScaling(1)
            .forwardZeroPowerAcceleration(-39.802429781401095)                                            // TODO: re-tune — run forward decel test
            .lateralZeroPowerAcceleration(-60.53661189108368)                                            // TODO: re-tune — run lateral decel test
            .translationalPIDFCoefficients(new PIDFCoefficients(.2, 0, 0.02, 0))         // TODO: re-tune translational PIDF
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.04, 0))                 // TODO: re-tune heading PIDF
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.00000001, 0.6, 0.04))        // TODO: re-tune drive PIDF
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 1, 2); // TODO: re-tune path constraints

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0.4840)                                                             // TODO: measure forward pod Y offset (inches)
            .strafePodX(-4.75)                                                              // TODO: measure strafe pod X offset (inches)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(77.32648714320865)                                                               // TODO: re-tune max forward velocity (in/s)
            .yVelocity(62.50220928041955)                                                               // TODO: re-tune max lateral velocity (in/s)
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
