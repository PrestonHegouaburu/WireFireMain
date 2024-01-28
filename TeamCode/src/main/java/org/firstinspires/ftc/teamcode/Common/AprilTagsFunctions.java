package org.firstinspires.ftc.teamcode.Common;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagsFunctions {
    private LinearOpMode lom;
    public CircleDetection circleDetection;
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private boolean isCameraReady = false;
    public static final int TAG_BLUE_LEFT = 1;
    public static final int TAG_BLUE_CENTER = 2;
    public static final int TAG_BLUE_RIGHT = 3;
    public static final int TAG_RED_LEFT = 4;
    public static final int TAG_RED_CENTER = 5;
    public static final int TAG_RED_RIGHT = 6;
    // Used to hold the data for a detected AprilTag
    public AprilTagDetection detectedTag = null;
    public AprilTagsFunctions(LinearOpMode l, boolean isRed) {
        lom = l;
        try {
            Initialize(isRed);
        }
        catch (Exception e) {
            isCameraReady = false;
        }
    }
    private void Initialize(boolean isRed) {
        circleDetection = new CircleDetection(isRed);
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(lom.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(aprilTag, circleDetection)
                .setCameraResolution(new Size(864, 480))
                .enableLiveView(false)
                .build();

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
    }
    public boolean DetectAprilTag(int desiredTag) {
        if(!isCameraReady)
            return false;
        boolean targetFound = false;
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((desiredTag < 0) || (detection.id == desiredTag)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    detectedTag = detection;
                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    //lom.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                //lom.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        //lom.telemetry.update();
        return targetFound;
    }
    public void RunCircleProcessorOnly()
    {
        visionPortal.setProcessorEnabled(circleDetection, true);
        visionPortal.setProcessorEnabled(aprilTag, false);
    }
    public void RunAprilTagProcessorOnly()
    {
        visionPortal.setProcessorEnabled(circleDetection, false);
        visionPortal.setProcessorEnabled(aprilTag, true);
    }
    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls
        if (visionPortal == null)
            return;
        ElapsedTime runtime = new ElapsedTime();
        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            lom.telemetry.addData("Camera", "Waiting");
            lom.telemetry.update();
            double startTime = runtime.milliseconds();
            while (!lom.isStopRequested() && ((runtime.milliseconds() - startTime) < 4000) &&
                    (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                lom.sleep(20);
            }
            lom.telemetry.addData("Camera", "Ready");
            lom.telemetry.update();
        }
        if(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
            return;

        // Set camera controls unless we are stopping.
        if (!lom.isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                lom.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            lom.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            lom.sleep(20);
        }
        isCameraReady = true;
    }
    public void UpdateCircleDetectionTelemetry(int tries)
    {
        lom.telemetry.addData("Tries", tries);
        lom.telemetry.addData("Frames processed", circleDetection.FramesProcessed());
        lom.telemetry.addData("Total No Detection", circleDetection.totalNoDetection);
        lom.telemetry.addData("Total One Circle", circleDetection.totalOneCircle);
        lom.telemetry.addData("Total Many Circles", circleDetection.totalManyCircles);
        lom.telemetry.addData("Total Left", circleDetection.totalLeft);
        lom.telemetry.addData("Total Center", circleDetection.totalCenter);
        lom.telemetry.addData("Total Right", circleDetection.totalRight);
        lom.telemetry.addData("FPS", String.format("%.2f", visionPortal.getFps()));
        lom.telemetry.addData("Circles detected", "%d", circleDetection.NumCirclesFound());
        lom.telemetry.addData("Circle center", "%4.0f, %4.0f", circleDetection.CircleCenter().x, circleDetection.CircleCenter().y);
        lom.telemetry.addData("Circle radius", "%4.2f", circleDetection.CircleRadius());
        lom.telemetry.addData("Ball Position", "%s", circleDetection.GetBallPosition());
        lom.telemetry.update();
    }
}
