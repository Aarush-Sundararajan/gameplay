package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "Red Bucket Vision")
public class ShooterAlignTest extends OpMode {

    private static final int TARGET_TAG_ID = 24;

    AprilTagWebcamSetup aprilTagWebcam = new AprilTagWebcamSetup();

    // AprilTag values (ALWAYS UPDATED)
    public boolean tagVisible = false;


    public double bearing = 0;

    public double startBearing = -2.0;
    public double endBearing = 2.0;
    public boolean bearingAligned = false;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        aprilTagWebcam.update();

        AprilTagDetection detection =
                aprilTagWebcam.getTagBySpecificID(TARGET_TAG_ID);

        if (detection != null) {
            tagVisible = true;

            double bearing = detection.ftcPose.bearing;

            if (bearing < startBearing) {
                // TURN LEFT SLOWLY
            } else if (bearing > endBearing) {
                // TURN RIGHT SLOWLY
            } else if (bearing >= startBearing && bearing <= endBearing) {
                //shoot
            }


        } else {
            tagVisible = false;
            telemetry.addLine("Not Detected");
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        aprilTagWebcam.stop();
    }
}
