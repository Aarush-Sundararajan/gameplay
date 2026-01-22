package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="Teleop_DoesNothing", group="TeleOp")
public class Teleop_DoesNothing extends LinearOpMode {

    // Drivetrain motors
    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    // Other hardware
    private DcMotor shootLeft, shootRight, intake;
    private Servo s1, s3;
    private CRServo s2, s4, s5, s6;

    // Vision
    private AprilTagWebcamSetup aprilTagWebcam;

    // Pinpoint for orientation
    private com.qualcomm.hardware.gobilda.GoBildaPinpointDriver pinpoint;

    // Teleop variables
    private static final double DEADZONE = 0.1;
    private static final double THRESH_WM_POWER = 0.8;
    private static final double kP_AprilTag = 0.015; // PID proportional constant for tag alignment
    private static final double maxTurnPower = 0.5;
    private static final double YAW_DEADZONE = 2.0; // degrees, prevents twitching

    // Helper
    private double clampDeadzone(double val) {
        return Math.abs(val) < DEADZONE ? 0 : val;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware Initialization ---
        frontLeftDrive = hardwareMap.get(DcMotor.class, "lf");
        backLeftDrive = hardwareMap.get(DcMotor.class, "lb");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backRightDrive = hardwareMap.get(DcMotor.class, "rr");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shootLeft = hardwareMap.get(DcMotor.class,"sLeft");
        shootRight = hardwareMap.get(DcMotor.class,"sRight");
        intake = hardwareMap.get(DcMotor.class,"intake");

        s1 = hardwareMap.get(Servo.class,"s1");
        s2 = hardwareMap.get(CRServo.class,"s2");
        s3 = hardwareMap.get(Servo.class,"s3");
        s4 = hardwareMap.get(CRServo.class,"s4");
        s5 = hardwareMap.get(CRServo.class,"s5");
        s6 = hardwareMap.get(CRServo.class,"s6");

        // Pinpoint
        pinpoint = hardwareMap.get(com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.resetPosAndIMU();

        // Vision
        aprilTagWebcam = new AprilTagWebcamSetup();
        aprilTagWebcam.init(hardwareMap, telemetry);

        telemetry.addLine("TeleOp initialized!");
        telemetry.update();
        waitForStart();

        // --- Main loop ---
        while(opModeIsActive()) {

            // --- Driver input ---
            double forward = -clampDeadzone(gamepad1.left_stick_y);
            double strafe = clampDeadzone(gamepad1.left_stick_x);

            // --- Vision update ---
            aprilTagWebcam.update();
            List<AprilTagDetection> detections = aprilTagWebcam.getDetectedTags();
            AprilTagDetection tagOfInterest = null;

            for(AprilTagDetection tag : detections) {
                if(tag.id == 24 || tag.id == 20){
                    tagOfInterest = tag;
                    break;
                }
            }

            double turnPower = 0;

            // Only apply auto-turn if button is pressed
            if(tagOfInterest != null && gamepad1.y){
                // Angle to tag from robot yaw
                double yawToTag = tagOfInterest.ftcPose.yaw; // degrees

                // Apply deadzone
                if(Math.abs(yawToTag) > YAW_DEADZONE){
                    turnPower = -yawToTag * kP_AprilTag;
                    turnPower = Range.clip(turnPower,-maxTurnPower,maxTurnPower);
                } else {
                    turnPower = 0;
                }

                telemetry.addData("Tag ID",tagOfInterest.id);
                telemetry.addData("Yaw to Tag", yawToTag);
                telemetry.addData("Turn Power", turnPower);
            }

            // --- Mecanum drive calculation ---
            double fl = forward + strafe + turnPower;
            double bl = forward - strafe + turnPower;
            double fr = forward - strafe - turnPower;
            double br = forward + strafe - turnPower;

            // Normalize to max power
            double max = Math.max(Math.max(Math.abs(fl), Math.abs(bl)), Math.max(Math.abs(fr), Math.abs(br)));
            if(max > 1){
                fl/=max; bl/=max; fr/=max; br/=max;
            }

            frontLeftDrive.setPower(fl);
            backLeftDrive.setPower(bl);
            frontRightDrive.setPower(fr);
            backRightDrive.setPower(br);

            // --- Teleop other controls ---
            if(gamepad2.y) intake.setPower(1);
            else if(gamepad2.x) intake.setPower(0);

            if(gamepad2.a){
                shootLeft.setPower(-1);
                shootRight.setPower(1);
            } else if(gamepad2.b){
                shootLeft.setPower(1);
                shootRight.setPower(-1);
            } else{
                shootLeft.setPower(0);
                shootRight.setPower(0);
            }

            if(gamepad2.right_bumper){
                s1.setPosition(0.5);
            } else if(gamepad2.left_bumper){
                s1.setPosition(-0.5);
            }

            if(gamepad2.right_trigger > 0.1){
                s3.setPosition(0.5);
            } else if(gamepad2.left_trigger > 0.1){
                s3.setPosition(0.7);
            }

            telemetry.update();
        }

        aprilTagWebcam.stop();
    }
}
