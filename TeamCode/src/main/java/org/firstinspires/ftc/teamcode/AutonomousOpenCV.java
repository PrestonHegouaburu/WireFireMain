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
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.55;
    private int desiredTag = 0;
    private double backDropDirection = 90.0;
    private void Initialize() {
        df = new DrivingFunctions(this);
        mf = new MotorFunctions(this);
        sf = new ServoFunctions(this, df, mf);
        af = new AprilTagsFunctions(this, isRed);
        af.RunCircleProcessorOnly();
    }
    private CircleDetection.BallPosition DetectBallPosition(int timeoutInSeconds) {
        int tries = 0;
        CircleDetection.BallPosition bp;
        while (opModeIsActive() && tries++ < timeoutInSeconds * 10 && !af.circleDetection.CircleFound(10)) {
            sleep(100);
            af.UpdateCircleDetectionTelemetry(tries);
        }
        bp = af.circleDetection.GetBallPosition();
        if (bp == CircleDetection.BallPosition.UNDEFINED)
             bp = CircleDetection.BallPosition.LEFT; // Ball not found, makes a guess to the left
        af.RunAprilTagProcessorOnly();
        return bp;
    }
    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();
        af.circleDetection.ResetTotals();
        df.ResetYaw();
        runtime.reset();
        if(RunningTests())
            return;

        CircleDetection.BallPosition bp = DetectBallPosition(3);

        double horizontalInchesFromBackdropCenter = 0;
        double strafeCorrection = 0;

        if(bp == CircleDetection.BallPosition.LEFT) {
            desiredTag = isRed ? AprilTagsFunctions.TAG_RED_LEFT : AprilTagsFunctions.TAG_BLUE_LEFT;
            strafeCorrection = PushPixelSide(false);
            horizontalInchesFromBackdropCenter = -7;
        } else if(bp == CircleDetection.BallPosition.CENTER) {
            desiredTag = isRed ? AprilTagsFunctions.TAG_RED_CENTER : AprilTagsFunctions.TAG_BLUE_CENTER;
            PushPixelCenter();
            horizontalInchesFromBackdropCenter = 0;
        } else { // Right
            desiredTag = isRed ? AprilTagsFunctions.TAG_RED_RIGHT : AprilTagsFunctions.TAG_BLUE_RIGHT;
            strafeCorrection = PushPixelSide(true);
            horizontalInchesFromBackdropCenter = 7;
        }
        backDropDirection = isRed ? -90.0 : 90.0;

        if(isNear)
            DriveToBackDropFromNearSide(strafeCorrection, horizontalInchesFromBackdropCenter);
        else
            DriveToBackDropFromFarSide(bp, strafeCorrection, horizontalInchesFromBackdropCenter);

        DeliverPixel();
        ParkRobot(horizontalInchesFromBackdropCenter);
    }
    private void PushPixelCenter() {
        // Ends in the center, 6" forward from starting point
        df.DriveStraight(DRIVE_SPEED,24 , 0, false);
        df.DriveStraight(DRIVE_SPEED * 0.3,6 , 0, false);
        df.DriveStraight(DRIVE_SPEED, -24, 0, false);
    }
    protected double PushPixelSide(boolean isRight) {
        // Ends in the center, 6" forward from starting point (shifted by strafeCorrection)
        double angle;
        double strafeCorrection = 0.0;
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
        df.DriveStraight(DRIVE_SPEED, movingAwayFromTruss ? 12 : 18 , angle, false);
        df.DriveStraight(DRIVE_SPEED * 0.3, 8, angle, false);
        df.DriveStraight(DRIVE_SPEED, movingAwayFromTruss ? -20 : -26, angle, false);
        df.TurnToHeading(TURN_SPEED,0);
        if (!movingAwayFromTruss) // if it strafed, it returns the distance it did, for later correction
            strafeCorrection = 10.0;
        return strafeCorrection;
    }
    private void DriveToBackDropFromNearSide(double strafeCorrection, double horizontalInchesFromBackdropCenter) {
        // Ends aligned with the proper AprilTag, 11" away from the backdrop
        df.DriveStraight(DRIVE_SPEED, isRed ? 30  - strafeCorrection : -30 + strafeCorrection, 0, true);
        df.DriveStraight(DRIVE_SPEED, isRed ? 19 - horizontalInchesFromBackdropCenter : 19 + horizontalInchesFromBackdropCenter, 0, false);
        df.TurnToHeading(TURN_SPEED, backDropDirection);
    }
    private void DriveToBackDropFromFarSide(CircleDetection.BallPosition bp, double strafeCorrection, double horizontalInchesFromBackdropCenter) {
        // Ends aligned with the proper AprilTag, 11" away from the backdrop
        double horizontalMove = bp == CircleDetection.BallPosition.CENTER ? 20 : 0.0;
        df.DriveStraight(DRIVE_SPEED, isRed ? -horizontalMove : horizontalMove, 0, true);
        df.DriveStraight(DRIVE_SPEED, 42, 0, false);
        df.TurnToHeading(TURN_SPEED, backDropDirection);
        // Wait until there are 13 seconds left
        long timeToWaitMilliseconds = 30000 - (long) runtime.milliseconds() - 13000;
        timeToWaitMilliseconds  = timeToWaitMilliseconds < 0 ? 0 : timeToWaitMilliseconds;
        telemetry.addData("Wait this number of milliseconds", timeToWaitMilliseconds);
        telemetry.update();
        sleep(timeToWaitMilliseconds);

        df.DriveStraight(DRIVE_SPEED * 1.4, 70 + horizontalMove + Math.abs(strafeCorrection), backDropDirection, false);
        df.DriveStraight(DRIVE_SPEED, isRed ? 27 + horizontalInchesFromBackdropCenter : -27 + horizontalInchesFromBackdropCenter, backDropDirection, true);
    }
    protected void DeliverPixel()
    {
        // Assumes that the robot is facing the backdrop, aligned with the correct AprilTag, 16 inches from the backdrop
        // If it is near, assumes it'll get there first, so it delivers on the first row of the backdrop.
        // If it is coming from the far side, it assumes there is a pixel from the other team already there, so it delivers in the second row (risky because it can bounce)
        int rowTarget = isNear ? 0 : 1;

        if(!df.DriveToAprilTagAutonomous(af, backDropDirection, desiredTag, sf.IdealDistanceFromBackdropToDeliver(rowTarget), DRIVE_SPEED)) {
            // if the alignment through AprilTag did not complete, it uses the distance sensor to finish the approach
            double dist = df.GetDistanceFromSensorInInches(1.0, 20.0);
            if (dist > 0.0)
                // Distance sensor worked fine
                dist = dist - sf.IdealDistanceFromBackdropToDeliver(rowTarget);
            else
                // Distance sensor didn't work, makes its best guess at the distance left
                dist = 14;

            df.DriveStraight(DRIVE_SPEED, dist, backDropDirection, false);
        }
        sf.PutPixelOnBackDrop(rowTarget);
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
            df.DriveStraight(DRIVE_SPEED, isRed ? -26-horizontalCorrection : 26-horizontalCorrection, backDropDirection, true);
        //Pushes against the wall to end parking
        df.DriveStraight(DRIVE_SPEED, 14, backDropDirection, false);
        df.DriveStraight(DRIVE_SPEED/2, 4, backDropDirection, false);
        if(this.cornerPark)
            df.DriveStraight(DRIVE_SPEED/3, isRed ? 4 : -4, backDropDirection, true);
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