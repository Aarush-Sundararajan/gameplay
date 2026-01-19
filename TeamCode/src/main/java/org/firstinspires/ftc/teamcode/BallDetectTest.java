package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;

@Autonomous(name = "Ball Alignment Test")
public class BallDetectTest extends LinearOpMode {

    OpenCvWebcam webcam;
    BallPipeline pipeline;

    /* ===== CAMERA CONSTANTS ===== */
    private static final double IMAGE_CENTER_X = 320.0; // 640 / 2
    private static final double DEADZONE_PIXELS = 10.0;  // alignment tolerance

    @Override
    public void runOpMode() {

        @SuppressLint("DiscouragedApi")
        int cameraMonitorViewId =
                hardwareMap.appContext.getResources()
                        .getIdentifier("cameraMonitorViewId", "id",
                                hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Camera 1"),
                cameraMonitorViewId
        );

        pipeline = new BallPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        waitForStart();

        while (opModeIsActive()) {

            if (pipeline.ballDetected) {

                double error = pipeline.ballCenterX - IMAGE_CENTER_X;

                telemetry.addData("Ball Center X", pipeline.ballCenterX);
                telemetry.addData("Error (px)", error);
                telemetry.addData("Ball Area", pipeline.ballArea);

                if (Math.abs(error) < DEADZONE_PIXELS) {
                    telemetry.addLine("ALIGNED");

                    // 🔒 STOP STRAFING
                    // strafe(0);

                } else {
                    // 🔁 STRAFE LEFT / RIGHT
                    // double strafePower = error * STRAFE_GAIN;
                    // strafe(strafePower);
                }

            } else {
                telemetry.addLine("No Ball Detected");

                // strafe(0);
            }

            telemetry.update();
            sleep(40);
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();
    }
}
