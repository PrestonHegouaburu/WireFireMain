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
    public enum BallPosition {LEFT, CENTER, RIGHT, UNDEFINED};
    private CircleDetection.BallPosition ballPosition = CircleDetection.BallPosition.UNDEFINED;
    private boolean detectionRed = true;
    private Mat grayMat = new Mat();
    private Mat hsvMaskedMat = new Mat();
    private Mat mask = new Mat();
    private Mat mask1 = new Mat();
    private Mat mask2 = new Mat();
    private Mat hsvMat = new Mat();
    private Mat subMat = new Mat();
    private double[] data;
    private int framesProcessed = 0;
    private int numCirclesFound = 0;
    private Point circleCenter = new Point(0.0, 0.0);

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

    public boolean CircleFound()
    {
        return ballPosition != CircleDetection.BallPosition.UNDEFINED;
    }
    public CircleDetection.BallPosition GetBallPosition()
    {
        return ballPosition;
    }

    public void SetBallPosition(CircleDetection.BallPosition ballPosition)
    {
        this.ballPosition = ballPosition;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        framesProcessed++;
        subMat = input.submat(new Rect(0, 100, 864, 250));
        Imgproc.cvtColor(subMat, hsvMat, Imgproc.COLOR_RGB2HSV);
        if (detectionRed) {
            Core.inRange(hsvMat, new Scalar(0, 70, 50), new Scalar(10, 255, 255), mask1); // RED 1
            Core.inRange(hsvMat, new Scalar(160, 70, 50), new Scalar(180, 255, 255), mask2); // RED 2
            Core.bitwise_or(mask1, mask2, mask);
        } else {
            Core.inRange(hsvMat, new Scalar(92, 60, 0), new Scalar(123, 255, 255), mask); //BLUE
        }
        hsvMaskedMat.release();
        Core.bitwise_and(subMat, subMat, hsvMaskedMat, mask);

        Imgproc.cvtColor(hsvMaskedMat, grayMat, Imgproc.COLOR_RGB2GRAY);

        Imgproc.GaussianBlur(grayMat, grayMat, new org.opencv.core.Size(15.0, 15.0), 2, 2);
        Mat circles = new Mat();
        Imgproc.HoughCircles(grayMat, circles, Imgproc.HOUGH_GRADIENT, 1, 300, 120, 25);

        numCirclesFound = circles.cols();

        for (int i = 0; i < numCirclesFound; i++) {
            data = circles.get(0, i);
            circleCenter = new Point(Math.round(data[0]), Math.round(data[1])+100);
            ballPosition = circleCenter.x < 250 ? CircleDetection.BallPosition.LEFT : (circleCenter.x > 600 ? CircleDetection.BallPosition.RIGHT : CircleDetection.BallPosition.CENTER);
        }
        return grayMat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        // do nothing
    }
}
