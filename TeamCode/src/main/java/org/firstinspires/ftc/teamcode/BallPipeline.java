package org.firstinspires.ftc.teamcode;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BallPipeline extends OpenCvPipeline {

    /* ===== OUTPUTS (READ IN OPMODE) ===== */
    public boolean ballDetected = false;
    public double ballCenterX = -1;   // pixel X of closest ball
    public double ballArea = 0;        // larger = closer

    /* ===== INTERNAL MATS ===== */
    private final Mat gray = new Mat();
    private final Mat blurred = new Mat();
    private final Mat thresh = new Mat();

    /* ===== TUNABLE VALUES ===== */

    // Minimum size to count as a ball
    private static final double MIN_BALL_AREA = 1200.0;

    // Threshold for brightness (ball vs background)
    private static final double THRESH_VALUE = 80.0;

    @Override
    public Mat processFrame(Mat input) {

        // Reset outputs every frame
        ballDetected = false;
        ballCenterX = -1;
        ballArea = 0;

        /* 1️⃣ Convert to grayscale */
        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);

        /* 2️⃣ Blur to remove noise + whiffle holes */
        Imgproc.GaussianBlur(gray, blurred, new Size(7, 7), 0);

        /* 3️⃣ Binary threshold */
        Imgproc.threshold(
                blurred,
                thresh,
                THRESH_VALUE,
                255,
                Imgproc.THRESH_BINARY
        );

        /* 4️⃣ Morphology (fill holes) */
        Mat kernel = Imgproc.getStructuringElement(
                Imgproc.MORPH_ELLIPSE,
                new Size(7, 7)
        );
        Imgproc.morphologyEx(thresh, thresh, Imgproc.MORPH_CLOSE, kernel);

        /* 5️⃣ Find contours */
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(
                thresh,
                contours,
                new Mat(),
                Imgproc.RETR_EXTERNAL,
                Imgproc.CHAIN_APPROX_SIMPLE
        );

        /* 6️⃣ Pick LARGEST contour = closest ball */
        MatOfPoint bestContour = null;
        double maxArea = 0;

        for (MatOfPoint c : contours) {
            double area = Imgproc.contourArea(c);
            if (area > MIN_BALL_AREA && area > maxArea) {
                maxArea = area;
                bestContour = c;
            }
        }

        /* 7️⃣ Extract alignment data */
        if (bestContour != null) {
            Rect rect = Imgproc.boundingRect(bestContour);

            ballDetected = true;
            ballArea = maxArea;
            ballCenterX = rect.x + rect.width / 2.0;

            // Draw box for debugging
            Imgproc.rectangle(
                    input,
                    rect,
                    new Scalar(0, 255, 0),
                    2
            );
        }

        return input;
    }
}
