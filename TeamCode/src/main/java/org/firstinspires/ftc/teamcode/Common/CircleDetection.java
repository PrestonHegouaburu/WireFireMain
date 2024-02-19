package org.firstinspires.ftc.teamcode.Common;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class CircleDetection implements VisionProcessor {
    public enum BallPosition {LEFT, CENTER, RIGHT, UNDEFINED}
    private final boolean detectionRed;
    private final Mat hsvMaskedMat = new Mat();
    private final Mat mask = new Mat();
    private final Mat mask1 = new Mat();
    private final Mat mask2 = new Mat();
    private final Mat hsvMat = new Mat();
    private int framesProcessed = 0;
    private int numCirclesFound = 0;
    private double circleRadius = 0;
    private Point circleCenter = new Point(0.0, 0.0);
    public int totalNoDetection = 0, totalOneCircle = 0, totalLeft = 0, totalCenter = 0, totalRight = 0 ,totalManyCircles = 0;

    public CircleDetection(boolean detectionRed) {
        this.detectionRed = detectionRed;
    }

    public int FramesProcessed() { return framesProcessed; }
    public int NumCirclesFound()
    {
        return numCirclesFound;
    }
    public Point CircleCenter()
    {
        return circleCenter;
    }
    public double CircleRadius() { return circleRadius; }

    public boolean CircleFound(int minNumberOfTimesDetected)
    {
        return totalLeft >= minNumberOfTimesDetected || totalCenter >= minNumberOfTimesDetected || totalRight >= minNumberOfTimesDetected;
    }
    public BallPosition GetBallPosition()
    {
        if (totalLeft > totalCenter && totalLeft > totalRight)
            return BallPosition.LEFT;
        if (totalCenter > totalLeft && totalCenter > totalRight)
            return BallPosition.CENTER;
        if (totalRight > totalCenter && totalRight > totalLeft)
            return BallPosition.RIGHT;

        return BallPosition.UNDEFINED;
    }
    public void ResetTotals()
    {
        totalRight = 0;
        totalLeft = 0;
        totalCenter = 0;
        totalNoDetection = 0;
        totalManyCircles = 0;
        totalOneCircle = 0;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        framesProcessed++;
        frame = frame.submat(new Rect(0, 100, 864, 250));
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        if (detectionRed) {
            Core.inRange(hsvMat, new Scalar(0, 70, 50), new Scalar(10, 255, 255), mask1); // RED 1
            Core.inRange(hsvMat, new Scalar(160, 70, 50), new Scalar(180, 255, 255), mask2); // RED 2
            Core.bitwise_or(mask1, mask2, mask);
        } else {
            Core.inRange(hsvMat, new Scalar(92, 60, 0), new Scalar(123, 255, 255), mask); //BLUE
        }
        hsvMaskedMat.release();
        Core.bitwise_and(frame, frame, hsvMaskedMat, mask);

        Imgproc.cvtColor(hsvMaskedMat, frame, Imgproc.COLOR_RGB2GRAY);

        Imgproc.GaussianBlur(frame, frame, new org.opencv.core.Size(15.0, 15.0), 2, 2);
        Mat circles = new Mat();
        Imgproc.HoughCircles(frame, circles, Imgproc.HOUGH_GRADIENT, 1, 300, 120, 25, 30, 70);

        numCirclesFound = circles.cols();
        double[] data;
        if (numCirclesFound == 1) {
            data = circles.get(0, 0);
            circleCenter = new Point(Math.round(data[0]), Math.round(data[1])+100);
            circleRadius = data[2];
            if(circleCenter.x < 250)
                totalLeft++;
            else if (circleCenter.x > 600)
                totalRight++;
            else
                totalCenter++;
            totalOneCircle++;
        }
        else if(numCirclesFound == 0)
            totalNoDetection++;
        else
            totalManyCircles++;

        return frame;
    }

    @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // do nothing
    }
}
