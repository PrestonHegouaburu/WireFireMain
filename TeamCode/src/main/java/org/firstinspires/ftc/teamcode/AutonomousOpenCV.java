package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.AprilTagsFunctions;
import org.firstinspires.ftc.teamcode.Common.CircleDetection;
import org.firstinspires.ftc.teamcode.Common.DrivingFunctions;
import org.firstinspires.ftc.teamcode.Common.MotorFunctions;
import org.firstinspires.ftc.teamcode.Common.ServoFunctions;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Autonomous(name="Autonomous - Generic", group="Linear Opmode")
@Disabled
public class AutonomousOpenCV extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private OpenCvWebcam webcam;
    protected CircleDetection circleDetection;
    protected DrivingFunctions df;
    protected AprilTagsFunctions aprilTagsFunctions;
    protected MotorFunctions mf;
    protected ServoFunctions sf;
    protected boolean isRed = false; // whether to detect a red ball (if false detects blue)
    protected boolean isNear = false; // whether we start from the near side of the backdrop
    protected boolean cornerPark = true; // whether we part on the corner or in the middle
    protected boolean runBallDetectionTest = false;
    static final double DRIVE_SPEED = 0.4;
    static final double TURN_SPEED = 0.5;
    private int desiredTag = 0;
    private double backDropDirection = 90.0;
    private void Initialize() {
        df = new DrivingFunctions(this);
        sf = new ServoFunctions(this, df);
        mf = new MotorFunctions(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        circleDetection = new CircleDetection(isRed);
        webcam.setPipeline(circleDetection);
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
    }
    private void DetectBallPosition(int timeoutInSeconds) {
        int tries = 0;
        while (opModeIsActive() && !circleDetection.CircleFound() && tries < timeoutInSeconds * 10) {
            sleep(100);
            tries++;
            UpdateCircleDetectionTelemetry(tries);
        }
        if (!circleDetection.CircleFound())
            circleDetection.SetBallPosition(CircleDetection.BallPosition.LEFT); // Ball not found, makes a guess to the left

        // After we are done detecting the ball, we switch the camera to use the AprilTags
        StopStreaming();
        aprilTagsFunctions = new AprilTagsFunctions(this);
    }
    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();
        runtime.reset();
        if(RunningTests())
            return;

        DetectBallPosition(5);

        double horizontalInchesFromBackdropCenter = 0;

        if(circleDetection.GetBallPosition() == CircleDetection.BallPosition.LEFT) {
            desiredTag = isRed ? AprilTagsFunctions.TAG_RED_LEFT : AprilTagsFunctions.TAG_BLUE_LEFT;
            PushPixelSide(false);
            horizontalInchesFromBackdropCenter = -7;
        } else if(circleDetection.GetBallPosition() == CircleDetection.BallPosition.CENTER) {
            desiredTag = isRed ? AprilTagsFunctions.TAG_RED_CENTER : AprilTagsFunctions.TAG_BLUE_CENTER;
            PushPixelCenter();
            horizontalInchesFromBackdropCenter = 0;
        } else { // Right
            desiredTag = isRed ? AprilTagsFunctions.TAG_RED_RIGHT : AprilTagsFunctions.TAG_BLUE_RIGHT;
            PushPixelSide(true);
            horizontalInchesFromBackdropCenter = 7;
        }
        backDropDirection = isRed ? -90.0 : 90.0;

        if(isNear)
            DriveToBackDropFromNearSide();
        else
            DriveToBackDropFromFarSide();

        DeliverPixel(horizontalInchesFromBackdropCenter);
        ParkRobot(horizontalInchesFromBackdropCenter);
    }
    protected void PushPixelSide(boolean isRight) {
        // Ends in the center, 6" forward from starting point
        double angle;
        boolean movingAwayFromTruss = (isRight && isNear && isRed) || (isRight && !isNear && !isRed) ||
                (!isRight && !isNear && isRed) || (!isRight && isNear && !isRed);
        df.DriveStraight(DRIVE_SPEED, 6, 0, false);
        if(movingAwayFromTruss)
          angle = isRight ? -24 : 24;
        else
          angle = isRight ? -40 : 40;
        if (!movingAwayFromTruss)
            df.DriveStraight(DRIVE_SPEED, isRight ? -10 : 10, 0, true);
        df.TurnToHeading(TURN_SPEED,angle);
        df.DriveStraight(DRIVE_SPEED, movingAwayFromTruss ? 20 : 27, angle, false);
        df.DriveStraight(DRIVE_SPEED, movingAwayFromTruss ? -20 : -27, angle, false);
        df.TurnToHeading(TURN_SPEED,0);
        if (!movingAwayFromTruss) // if it strafed, now it goes back to the start position
            df.DriveStraight(DRIVE_SPEED, isRight ? 10 : -10, 0, true);
    }
    private void PushPixelCenter() {
        // Ends in the center, 6" forward from starting point
        df.DriveStraight(DRIVE_SPEED,31 , 0, false);
        df.DriveStraight(DRIVE_SPEED, -25, 0, false);
    }
    private void DriveToBackDropFromNearSide() {
        // Ends aligned with the center AprilTag, 11" away from the backdrop
        df.DriveStraight(DRIVE_SPEED, -3, 0, false);
        df.DriveStraight(DRIVE_SPEED, isRed ? 28 : -28, 0, true);
        df.DriveStraight(DRIVE_SPEED, 22, 0, false);
        df.TurnToHeading(TURN_SPEED, backDropDirection);
        df.DriveStraight(DRIVE_SPEED, 2, backDropDirection, false);
    }
    private void DriveToBackDropFromFarSide() {
        // Ends aligned with the center AprilTag, 11" away from the backdrop
        df.DriveStraight(DRIVE_SPEED, -3, 0, false);
        df.DriveStraight(DRIVE_SPEED, isRed ? -24 : 24, 0, true);
        df.DriveStraight(DRIVE_SPEED, 46, 0, false);
        df.TurnToHeading(TURN_SPEED, backDropDirection);
        // Wait until there are 10 seconds left
        long timeToWaitMilliseconds = 30000 - (long) runtime.milliseconds() - 14000;
        sleep(timeToWaitMilliseconds);
        df.DriveStraight(DRIVE_SPEED * 1.5, 100, backDropDirection, false);
        df.DriveStraight(DRIVE_SPEED, isRed ? 26 : -26, backDropDirection, true);
    }
    protected void DeliverPixel(double horizontalInchesFromBackdropCenter)
    {
        // Assumes that the robot is facing the backdrop, aligned with the middle AprilTag, 16 inches from the backdrop
        // If it is near, assumes it'll get there first, so it delivers on the first row of the backdrop.
        // If it is coming from the far side, it assumes there is a pixel from the other team already there, so it delivers in the second row (risky because it can bounce)
        int targetRow = isNear ? 1 : 2;
        // Strafe to face desiredAprilTag
        df.DriveStraight(DRIVE_SPEED, horizontalInchesFromBackdropCenter, backDropDirection, true);
        if(!df.DriveToAprilTag(aprilTagsFunctions, backDropDirection, desiredTag, 0, sf.IdealDistanceFromBackdropToDeliver(targetRow), DRIVE_SPEED)) {
            // if the alignment through AprilTag did not complete, it uses the distance sensor to finish the approach
            double dist = df.GetDistanceFromSensorInInches();
            if (dist > 0.0 && dist < 30.0) {
                dist = dist - sf.IdealDistanceFromBackdropToDeliver(targetRow);
                df.DriveStraight(DRIVE_SPEED, dist, backDropDirection, false);
            }
        }
        mf.MoveSlidesToRowTarget(0.5, targetRow);
        sf.PutPixelOnBackDrop();
        mf.MoveSlidesToRowTarget(0.5, 0); // Bring the slides back down, to start TeleOp in zero
        // Gets away from the board after delivering pixel
        df.DriveStraight(DRIVE_SPEED, -6, backDropDirection, false);
    }
    protected void ParkRobot(double horizontalCorrection)
    {
        // Assumes that the robot just delivered the pixel and moved back from the backdrop a bit
        // But it still in front of where the pixel was dropped, so need to correct by horizontalCorrection
        if (this.cornerPark)
            df.DriveStraight(DRIVE_SPEED, isRed ? 26-horizontalCorrection : -26-horizontalCorrection, backDropDirection, true);
        else
            df.DriveStraight(DRIVE_SPEED, isRed ? -24-horizontalCorrection : 24-horizontalCorrection, backDropDirection, true);
        //Pushes against the wall to end parking
        df.DriveStraight(DRIVE_SPEED, 10, backDropDirection, false);
    }
    protected void finalize()
    {
        StopStreaming();
    }
    private void StopStreaming()
    {
        try {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
        catch (Exception e)
        {}
    }
    private void UpdateCircleDetectionTelemetry(int tries)
    {
        telemetry.addData("Tries: ", tries);
        telemetry.addData("Frame Count", webcam.getFrameCount());
        telemetry.addData("Frames processed: ", circleDetection.FramesProcessed());
        telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
        telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
        telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
        telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
        telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
        telemetry.addData("Circles detected: ", "%d", circleDetection.NumCirclesFound());
        telemetry.addData("Circle center = ", "%4.0f, %4.0f", circleDetection.CircleCenter().x, circleDetection.CircleCenter().y);
        telemetry.addData("Ball Position: ", "%s", circleDetection.GetBallPosition());
        telemetry.update();
    }
    private void RunBallDetectionTest() {
        while (opModeIsActive()) {
            sleep(100);
            UpdateCircleDetectionTelemetry(0);
        }
    }
    private boolean RunningTests()
    {
        if (runBallDetectionTest) {
            RunBallDetectionTest();
            return true;
        }
        return false;
    }
}