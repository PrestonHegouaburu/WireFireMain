package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.AprilTagsFunctions;
import org.firstinspires.ftc.teamcode.Common.DrivingFunctions;
import org.firstinspires.ftc.teamcode.Common.ServoFunctions;
import org.firstinspires.ftc.teamcode.Common.MotorFunctions;
@TeleOp(name="Wire Fire TeleOp Generic", group="TeleOp")
@Disabled
public class WireFireTeleOp extends LinearOpMode {
    private final ElapsedTime runtime = new ElapsedTime();
    private AprilTagsFunctions af = null;
    protected DrivingFunctions df = null;
    protected ServoFunctions sf = null;
    protected MotorFunctions mf = null;
    protected double intakeSpeed = 0.85;
    protected double speedFactor = 0.5; // Speed factor to slow down the robot, goes from 0.1 to 1.0
    private static final double slidesSpeed = 0.6;
    protected boolean IsTestMode = false;
    protected boolean isRedTeam = true;
    protected Gamepad currentGamepad1, previousGamepad1, currentGamepad2, previousGamepad2;
    private double x, y, yaw;
    private boolean isAutoTurning = false;
    private double autoTurningStart, autoTurningTarget, autoTurningTimeoutMilliseconds;
    private int columnTarget = 1; // from 1 to 7
    private int rowTarget = 0;
    private int actualSlidesRowPosition = 0;
    private int movingTowardRowPosition = -1; // -1 means that there is no slide movement now

    private int targetAprilTag;
    private void Init() {
        df = new DrivingFunctions(this);
        sf = new ServoFunctions(this, df);
        af = new AprilTagsFunctions(this, isRedTeam);
        af.RunAprilTagProcessorOnly();
        mf = new MotorFunctions(this);
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        currentGamepad2 = new Gamepad();
        previousGamepad2 = new Gamepad();
        columnTarget = 3;
        SetTargetAprilTag();
    }
    @Override
    public void runOpMode() {
        Init();
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            UpdateGamepad();
            CheckPlaneLaunch();
            RobotCentricDriving();
            FieldCentricDriving();
            AutoTurning();
            ProcessTestCommands();
            ProcessPixelDelivery();
            CheckRevertDirection();
            IntakeCommands();
            UpdateTelemetry();
            UpdateSlidesPosition();
            SetBackdropTargets();
            df.MoveRobot(x, y, yaw, speedFactor);
        }
    }
    private void UpdateSlidesPosition() {
        // The goal is to have the slide positions match the rowTarget variable
        // This function should end quickly so it doesn't block the rest of the commands
        if(actualSlidesRowPosition != rowTarget ||
                (movingTowardRowPosition != -1 && movingTowardRowPosition != rowTarget)) {
            mf.StartMovingSlidesToRowTarget(slidesSpeed, rowTarget);
            movingTowardRowPosition = rowTarget;
        }
        if (movingTowardRowPosition != -1 && mf.AreSlidesDoneMovingToTarget()) {
            movingTowardRowPosition = -1; // the slides stopped moving, as we arrived to the position
            actualSlidesRowPosition = rowTarget;
        }
    }

    private void IntakeCommands() {
        if (currentGamepad2.left_trigger > 0)
            mf.SetIntakePower(intakeSpeed);
        if (currentGamepad2.right_trigger > 0)
            mf.SetIntakePower(-intakeSpeed);
        if (currentGamepad2.left_trigger == 0 && currentGamepad2.right_trigger == 0)
            mf.SetIntakePower(0);

    }
    private void CheckRevertDirection() {
        if (previousGamepad1.left_trigger < 0.5 && currentGamepad1.left_trigger > 0.5) {
            if (df.isRobotDrivingForward())
                df.SetDirectionBackward();
            else
                df.SetDirectionForward();
        }
    }
    private void ProcessPixelDelivery() {
        if (!previousGamepad2.start && currentGamepad2.start && !currentGamepad2.b && !currentGamepad2.a)
            sf.PutPixelOnBackDrop();
        if (previousGamepad1.right_trigger < 0.5 && currentGamepad1.right_trigger > 0.5) {
            // Auto-aligning only works in the forward direction
            df.SetDirectionForward();
            double horizontalShift;
            if(rowTarget % 2 == 1)  // in odd rows the shift is 1.5 for even columns and -1.5 for odd columns
                horizontalShift = columnTarget % 2 == 0 ? 1.5 : -1.5;
            else // in even rows we have 0 shift in even columns, and -3 for odd, except for 7 it is 3
                horizontalShift = columnTarget == 7 ? 3.0 : (columnTarget % 2 == 0 ? 0.0 : -3.0);

            if(!df.DriveToAprilTag(af, 0.0, targetAprilTag, horizontalShift,sf.IdealDistanceFromBackdropToDeliver(rowTarget), 0.7))
                return;
            sf.PutPixelOnBackDrop();
            df.DriveStraight(0.6, -3.0, df.GetHeading(), false);
        }
    }
    private void SetBackdropTargets() {
        if(IsTestMode)
            return;

        int oldRowTarget = rowTarget;
        int oldColumnTarget = columnTarget;

        if (!previousGamepad2.left_bumper && currentGamepad2.left_bumper)
            if (rowTarget > 0)
                rowTarget--;

        if (!previousGamepad2.right_bumper && currentGamepad2.right_bumper)
            if (rowTarget < MotorFunctions.MAX_ROWS_OF_PIXELS)
                rowTarget++;

        if (oldRowTarget != rowTarget && columnTarget == 7)
            columnTarget = 6; // columnTarget cannot be 7 on two consecutive rows, as odd rows have 6 columns

        if (!previousGamepad2.x && currentGamepad2.x)
            columnTarget = columnTarget == 1 ? 2 : 1;

        if (!previousGamepad2.y && currentGamepad2.y)
            columnTarget = columnTarget == 3 ? 4 : 3;

        if (!previousGamepad2.b && currentGamepad2.b)
            columnTarget = columnTarget == 5 ? 6 : 5;

        if (!previousGamepad2.a && currentGamepad2.a)
            columnTarget = rowTarget % 2 == 0 ? 7 : 6;

        // Set targetAprilTag if the column target changed
        if (oldColumnTarget != columnTarget) {
            SetTargetAprilTag();
        }
    }
    private void SetTargetAprilTag() {
        int redAprilTagMapping[] = {AprilTagsFunctions.TAG_RED_LEFT, AprilTagsFunctions.TAG_RED_LEFT,
                AprilTagsFunctions.TAG_RED_CENTER, AprilTagsFunctions.TAG_RED_CENTER,
                AprilTagsFunctions.TAG_RED_RIGHT, AprilTagsFunctions.TAG_RED_RIGHT,AprilTagsFunctions.TAG_RED_RIGHT};
        int blueAprilTagMapping[] = {AprilTagsFunctions.TAG_BLUE_LEFT, AprilTagsFunctions.TAG_BLUE_LEFT,
                AprilTagsFunctions.TAG_BLUE_CENTER, AprilTagsFunctions.TAG_BLUE_CENTER,
                AprilTagsFunctions.TAG_BLUE_RIGHT, AprilTagsFunctions.TAG_BLUE_RIGHT,AprilTagsFunctions.TAG_BLUE_RIGHT};
        targetAprilTag = isRedTeam ? redAprilTagMapping[columnTarget -1] : blueAprilTagMapping[columnTarget -1];
    }

    private void RobotCentricDriving() {
        y = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        yaw = df.isRobotDrivingForward() ? gamepad1.right_stick_x : gamepad1.right_stick_x * -1;
        if (!previousGamepad1.left_bumper && currentGamepad1.left_bumper)
            speedFactor = 0.5;
        if (!previousGamepad1.right_bumper && currentGamepad1.right_bumper)
            speedFactor = 1;
        if (!previousGamepad1.back && currentGamepad1.back)
            df.ResetYaw();
    }
    private void AutoTurning() {
        double kp = -0.033;
        double botHeading = df.GetHeading();
        if(gamepad1.start)
            return;

        if (!isAutoTurning &&
                ((!previousGamepad1.y && currentGamepad1.y) || (!previousGamepad1.x && currentGamepad1.x) ||
                        (!previousGamepad1.b && currentGamepad1.b) || (!previousGamepad1.a && currentGamepad1.a))) {
            isAutoTurning = true;
            autoTurningTarget = currentGamepad1.y ? 0.0 : currentGamepad1.x ? 90.0 : currentGamepad1.b ? -90.0 : 179.9;
            autoTurningStart = runtime.milliseconds();
            double totalDeltaDegrees = (autoTurningTarget - botHeading + 540) % 360 - 180;
            autoTurningTimeoutMilliseconds = Math.abs(totalDeltaDegrees) / 180 * 3000 + 1000;
        }

        if (isAutoTurning) {
            double currentTime = runtime.milliseconds();
            double deltaDegrees = (autoTurningTarget - botHeading + 540) % 360 - 180;
            if ((Math.abs(deltaDegrees) < 1.0 && Math.abs(df.GetRotatingSpeed()) < 2.0) || (currentTime - autoTurningStart > autoTurningTimeoutMilliseconds)) {
                isAutoTurning = false;
            } else {
                yaw = kp * deltaDegrees / speedFactor;
            }
        }
    }
    protected void ProcessTestCommands() {
        return;
    }
    private void FieldCentricDriving() {
        // Field centric driving is activated when any of the hat buttons are pressed
        if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
            x = gamepad1.dpad_left ? -1 : (gamepad1.dpad_right ? 1 : 0);
            y = gamepad1.dpad_up ? 1 : (gamepad1.dpad_down ? -1 : 0);
            double botHeadingRadians = Math.toRadians(df.GetHeading());
            // Rotate the movement direction counter to the bot's rotation
            double newX = x * Math.cos(-botHeadingRadians) - y * Math.sin(-botHeadingRadians);
            double newY = x * Math.sin(-botHeadingRadians) + y * Math.cos(-botHeadingRadians);
            x = newX;
            y = newY;
        }
    }
    private void CheckPlaneLaunch() {
        if (!previousGamepad2.back && currentGamepad2.back) {
            // If we are playing the real game, only launch the plane in the last 30 seconds
            // (during "End game", after the first 90 minutes of regular Teleop passed)
            // From Game Manual 1: End Game – The last thirty seconds of the two-minute (2:00) Driver-Controlled Period
            if (!IsTestMode && runtime.seconds() < 90)
                return;
            sf.LaunchPlane();
        }
    }
    private void UpdateGamepad() {
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad2.copy(gamepad2);
    }
    protected void UpdateTelemetry() {
        telemetry.addData("TargetAprilTag", "%d", targetAprilTag);
        telemetry.addData("Detected AprilTag ID",  "%2d", af.DetectAprilTag(targetAprilTag) ? targetAprilTag : -1);
        telemetry.addData("Distance", "%4.2f", af.detectedTag != null ? af.detectedTag.ftcPose.range : -1);
        telemetry.addData("Driving Direction", df.isRobotDrivingForward() ? "Forward" : "Backward");
        telemetry.addData("Speed Factor", "%4.2f", speedFactor);
        telemetry.addData("Backdrop Row Target", "%2d", rowTarget);
        telemetry.addData("Backdrop Column", "%2d", columnTarget);
        telemetry.update();
    }
}

