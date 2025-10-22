package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//These are imports that allow you to use the multiple different tools the FIRST organization has created.


@Autonomous(name="Unit_Test", group="Linear OpMode")
public class Unit_test extends LinearOpMode {
    //The classifiers above are the boundaries for your code.
    //The name is how this code will be be titled on your FTC robot controller
    //The name next to "public class" should be the same as what you have named the file


    //This area of the code is a "setup" area where you initialize key variables and parts of the robot

    //In this area we need to setup our motors first
    //There should be 4 motors initialized like shown below.
    //As you add on additional motors they should be setup in the same pattern
    // (type of motor) (Name of motor) = null; - Most motors are DcMotors but make sure to check before coding
    // Leaving it null is essential so that motors ( and any other part of your robot ) doesn't start before you reach opMode
    DcMotor frontLeft = null;
    DcMotor backLeft = null;
    DcMotor frontRight = null;
    DcMotor backRight = null;

    //The 2nd main thing you will be using in FTC are servos
    //They are initialized using the same 'type variableName = null;
    // There are different types of Servos such as CRServos so be careful you are typing the correct type.

   Servo servo1 = null;

    //Now that your robot parts have initialized, its time to move into the runOpMode() section

    @Override
    public void runOpMode() {



        //This is the area where your motor variables are linked with your FTC robot controller
        //The format is as follows:
        // variableName = hardwareMap.get(classType.class,"deviceName");

        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        backLeft = hardwareMap.get(DcMotor.class, "back_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backRight = hardwareMap.get(DcMotor.class, "back_right");

        servo1 = hardwareMap.get(Servo.class, "Servo_1");


        //After mapping your motors and servos, set a direction for the motors/servos to spin
        //Direction.FORWARD means that when you power the motor by a positive value, the motor spins forward
        //Direction.REVERSE means that when you power the motor by a positive value the motor spins backwards
        //The left side motors should be in reverse mode because they are arranged in a opposite fashion to the right side
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        servo1.setDirection(Servo.Direction.FORWARD);

        //Telemetry is a way for you to send signals to your robot controller in the form of text
        //In this instance it is used to confirm that everything is initialized properly
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        //This block tells the code to wait until the start button is pressed before begining the code


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //This space is for actual robot movements:
            //Moving a motor
            frontLeft.setPower(1);
            //the powers range from 0.0 - 1.0
            servo1.setPosition(0.75);
            //servo positions are more complex with it only following a 270 degree movement cycle
            //You have to find hte specific position you want using a servo tester tool or use trial and error






        }
    }}

