package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.AprilTagsFunctions;
import org.firstinspires.ftc.teamcode.Common.CircleDetection;
import org.firstinspires.ftc.teamcode.Common.DrivingFunctions;
import org.firstinspires.ftc.teamcode.Common.MotorFunctions;
import org.firstinspires.ftc.teamcode.Common.ServoFunctions;

@Autonomous(name="Autonomous - Generic", group="Linear Opmode")
@Disabled
public class AutonomousOpenCV extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    protected DrivingFunctions df;
    protected AprilTagsFunctions af;
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
        af = new AprilTagsFunctions(this, isRed);
        af.RunCircleProcessorOnly();
    }
    private void DetectBallPosition(int timeoutInSeconds) {
        int tries = 0;
        while (opModeIsActive() && !af.circleDetection.CircleFound() && tries < timeoutInSeconds * 10) {
            sleep(100);
            tries++;
            af.UpdateCircleDetectionTelemetry(tries);
        }
        if (!af.circleDetection.CircleFound())
            af.circleDetection.SetBallPosition(CircleDetection.BallPosition.LEFT); // Ball not found, makes a guess to the left
        af.RunAprilTagProcessorOnly();
    }
    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();
        df.ResetYaw();
        runtime.reset();
        if(RunningTests())
            return;

        DetectBallPosition(5);

        double horizontalInchesFromBackdropCenter = 0;

        if(af.circleDetection.GetBallPosition() == CircleDetection.BallPosition.LEFT) {
            desiredTag = isRed ? AprilTagsFunctions.TAG_RED_LEFT : AprilTagsFunctions.TAG_BLUE_LEFT;
            PushPixelSide(false);
            horizontalInchesFromBackdropCenter = -7;
        } else if(af.circleDetection.GetBallPosition() == CircleDetection.BallPosition.CENTER) {
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
        long timeToWaitMilliseconds = 30000 - (long) runtime.milliseconds() - 15000;
        if (timeToWaitMilliseconds < 0)
            timeToWaitMilliseconds = 0;
        telemetry.addData("Wait this number of milliseconds", "%d", timeToWaitMilliseconds);
        telemetry.update();
        sleep(timeToWaitMilliseconds);
        df.DriveStraight(DRIVE_SPEED * 1.5, 100, backDropDirection, false);
        df.DriveStraight(DRIVE_SPEED, isRed ? 26 : -26, backDropDirection, true);
    }
    protected void DeliverPixel(double horizontalInchesFromBackdropCenter)
    {
        // Assumes that the robot is facing the backdrop, aligned with the middle AprilTag, 16 inches from the backdrop
        // If it is near, assumes it'll get there first, so it delivers on the first row of the backdrop.
        // If it is coming from the far side, it assumes there is a pixel from the other team already there, so it delivers in the second row (risky because it can bounce)
        int rowTarget = isNear ? 1 : 2;
        // Strafe to face desiredAprilTag
        df.DriveStraight(DRIVE_SPEED, horizontalInchesFromBackdropCenter, backDropDirection, true);
        if(!df.DriveToAprilTag(af, backDropDirection, desiredTag, 0, sf.IdealDistanceFromBackdropToDeliver(rowTarget), DRIVE_SPEED)) {
            // if the alignment through AprilTag did not complete, it uses the distance sensor to finish the approach
            double dist = df.GetDistanceFromSensorInInches(2.0, 20.0);
            if (dist > 0.0)
                // Distance sensor worked fine
                dist = dist - sf.IdealDistanceFromBackdropToDeliver(rowTarget);
            else
                // Distance sensor didn't work, makes its best guess at the distance left
                dist = 14;

            df.DriveStraight(DRIVE_SPEED, dist, backDropDirection, false);
        }
        mf.MoveSlidesToRowTargetSync(0.3, rowTarget);
        sf.PutPixelOnBackDrop();
        mf.MoveSlidesToRowTargetSync(0.3, 0); // Bring the slides back down, to start TeleOp in zero
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

    private void RunBallDetectionTest() {
        while (opModeIsActive()) {
            sleep(100);
            af.UpdateCircleDetectionTelemetry(0);
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