package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(2.9)
            .forwardZeroPowerAcceleration(-30.374416666486887)
            .lateralZeroPowerAcceleration(-41.47128955409912)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03,0,0.0001,0.018))
            .headingPIDFCoefficients(new PIDFCoefficients(0.4,0,0.002,0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.6,0.0,0.0001,0.6,0.025))
            .centripetalScaling(0.0012)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(62.95017533400949)
            .yVelocity(48.42991189664459);





    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("rr")
            .strafeEncoder_HardwareMapName("lr")
            .forwardEncoderDirection(Encoder.REVERSE)
            .forwardPodY(0.25)
            .strafePodX(0)
            .forwardTicksToInches(0.00206885)
            .strafeTicksToInches(0.0019754454)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                    )
            );
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1.4,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();

    }
}
