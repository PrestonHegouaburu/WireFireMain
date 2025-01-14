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
    protected double intakeSpeed = 1;
    protected double speedFactor = 0.75; // Speed factor to slow down the robot, goes from 0.1 to 1.0
    protected double speedFactorMin = 0.75;
    protected double speedFactorMax = 1;
    protected boolean IsTestMode = false;
    protected boolean isRedTeam = true;
    protected Gamepad currentGamepad1, previousGamepad1, currentGamepad2, previousGamepad2;
    private double x, y, yaw;
    private boolean isAutoTurning = false;
    private double autoTurningStart, autoTurningTarget, autoTurningTimeoutMilliseconds;
    private int columnTarget = 1; // from 1 to 6
    private int rowTarget = 2;
    private int targetAprilTag;
    private boolean areSlidesMovingManually = false;
    private void Init() {
        df = new DrivingFunctions(this);
        mf = new MotorFunctions(this, this);
        sf = new ServoFunctions(this, df, mf, this);
        af = new AprilTagsFunctions(this, isRedTeam);
        af.RunAprilTagProcessorOnly();
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
            BasicDriving();
            CheckPlaneLaunch();
            AutoTurning();
            ProcessTestCommands();
            ProcessPixelDelivery();
            IntakeCommands();
            UpdateTelemetry();
            UpdateSlidesPosition();
            SetBackdropTargets();
            df.MoveRobot(x, y, yaw, speedFactor);
        }
    }
    public void ActiveSleep(int milliseconds)
    {
        double sleepStart = runtime.milliseconds();

        while(runtime.milliseconds() - sleepStart <= milliseconds) {
            BasicDriving();
            df.MoveRobot(x, y, yaw, speedFactor);
        }
    }
    private void BasicDriving() {
        UpdateGamepad();
        RobotCentricDriving();
        FieldCentricDriving();
    }
    private void UpdateSlidesPosition() {
        if(areSlidesMovingManually && !gamepad2.dpad_down && !gamepad2.dpad_up) {
            areSlidesMovingManually = false;
            mf.MoveSlides(0);
        }
        if(gamepad2.dpad_up) {
            areSlidesMovingManually = true;
            mf.MoveSlides(0.6);
        }
        if(gamepad2.dpad_down) {
            areSlidesMovingManually = true;
            mf.MoveSlides(-0.6);
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
    private void ProcessPixelDelivery() {
        if (!previousGamepad2.back && currentGamepad2.back && !currentGamepad2.b && !currentGamepad2.a)
             sf.PutPixelOnBackDrop(rowTarget, true);

        if ((!previousGamepad2.start && currentGamepad2.start) || (previousGamepad1.right_trigger < 0.5 && currentGamepad1.right_trigger > 0.5)){
            double horizontalShift = columnTarget % 2 == 0 ? 1 : -1;
            if(!df.DriveToAprilTagTeleop(af, 0.0, targetAprilTag, horizontalShift,sf.IdealDistanceFromBackdropToDeliver(rowTarget), 1.0))
                return;
            sf.PutPixelOnBackDrop(rowTarget, false);
            df.DriveStraight(0.8, -6.0, df.GetHeading(), false);
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

        // Set targetAprilTag if the column target changed
        if (oldColumnTarget != columnTarget) {
            SetTargetAprilTag();
        }
    }
    private void SetTargetAprilTag() {
        int[] redAprilTagMapping = {AprilTagsFunctions.TAG_RED_LEFT, AprilTagsFunctions.TAG_RED_LEFT,
                AprilTagsFunctions.TAG_RED_CENTER, AprilTagsFunctions.TAG_RED_CENTER,
                AprilTagsFunctions.TAG_RED_RIGHT, AprilTagsFunctions.TAG_RED_RIGHT,AprilTagsFunctions.TAG_RED_RIGHT};
        int[] blueAprilTagMapping = {AprilTagsFunctions.TAG_BLUE_LEFT, AprilTagsFunctions.TAG_BLUE_LEFT,
                AprilTagsFunctions.TAG_BLUE_CENTER, AprilTagsFunctions.TAG_BLUE_CENTER,
                AprilTagsFunctions.TAG_BLUE_RIGHT, AprilTagsFunctions.TAG_BLUE_RIGHT,AprilTagsFunctions.TAG_BLUE_RIGHT};
        targetAprilTag = isRedTeam ? redAprilTagMapping[columnTarget -1] : blueAprilTagMapping[columnTarget -1];
    }
    private void RobotCentricDriving() {
        y = gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        yaw = gamepad1.right_stick_x;
        if (!previousGamepad1.left_bumper && currentGamepad1.left_bumper)
            speedFactor = speedFactorMin;
        if (!previousGamepad1.right_bumper && currentGamepad1.right_bumper)
            speedFactor = speedFactorMax;
        if (!previousGamepad1.back && currentGamepad1.back)
            df.ResetYaw();
    }

    private boolean TriggerAutoRotate()
    {
        boolean triggerAutoRotate = ((!previousGamepad1.y && currentGamepad1.y) || (!previousGamepad1.x && currentGamepad1.x) ||
                (!previousGamepad1.b && currentGamepad1.b) || (!previousGamepad1.a && currentGamepad1.a));
        triggerAutoRotate = triggerAutoRotate || (currentGamepad1.y && currentGamepad1.b) ||
                (currentGamepad1.x && currentGamepad1.y) || (currentGamepad1.a && currentGamepad1.b) ||
                (currentGamepad1.x && currentGamepad1.a);
        return triggerAutoRotate;
    }
    private void AutoTurning() {
        double kp = -0.033;
        double botHeading = df.GetHeading();
        if(gamepad1.start)
            return;
        if (TriggerAutoRotate()) {
            isAutoTurning = true;
            if (currentGamepad1.y && currentGamepad1.b)
                autoTurningTarget = 45;
            else if (currentGamepad1.x && currentGamepad1.y)
                autoTurningTarget = 135;
            else if (currentGamepad1.a && currentGamepad1.b)
                autoTurningTarget = -45;
            else if(currentGamepad1.x && currentGamepad1.a)
                autoTurningTarget = -135;
            else if(currentGamepad1.y)
                autoTurningTarget = 90;
            else if(currentGamepad1.x)
                autoTurningTarget = 180;
            else if(currentGamepad1.b)
                autoTurningTarget = 0;
            else // this is A
                autoTurningTarget = -90;

            autoTurningTarget = isRedTeam ? autoTurningTarget + 180 : autoTurningTarget;
            autoTurningStart = runtime.milliseconds();
            double totalDeltaDegrees = (autoTurningTarget - botHeading + 540) % 360 - 180;
            autoTurningTimeoutMilliseconds = Math.abs(totalDeltaDegrees) / 180 * 3000 + 350;
        }
        if (isAutoTurning) {
            double currentTime = runtime.milliseconds();
            double deltaDegrees = (autoTurningTarget - botHeading + 540) % 360 - 180;
            if ((Math.abs(deltaDegrees) < 2.0 && Math.abs(df.GetRotatingSpeed()) < 4.0) || (currentTime - autoTurningStart > autoTurningTimeoutMilliseconds)) {
                isAutoTurning = false;
            } else {
                yaw = kp * deltaDegrees / speedFactor;
            }
        }
    }
    protected void ProcessTestCommands() {
    }
    private void FieldCentricDriving() {
        // Field centric driving is activated when any of the hat buttons are pressed
        if (gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_left || gamepad1.dpad_right) {
            if (isRedTeam){
                x = gamepad1.dpad_down ? 1 : (gamepad1.dpad_up ? -1 : 0);
                y = gamepad1.dpad_left ? -1 : (gamepad1.dpad_right ? 1 : 0);
            } else {
                x = gamepad1.dpad_down ? -1 : (gamepad1.dpad_up ? 1 : 0);
                y = gamepad1.dpad_left ? 1 : (gamepad1.dpad_right ? -1 : 0);
            }

            double botHeadingRadians = Math.toRadians(df.GetHeading());
            // Rotate the movement direction counter to the bot's rotation
            double newX = x * Math.cos(-botHeadingRadians) - y * Math.sin(-botHeadingRadians);
            double newY = x * Math.sin(-botHeadingRadians) + y * Math.cos(-botHeadingRadians);
            x = newX;
            y = newY;
        }
    }
    private void CheckPlaneLaunch() {
        if (!previousGamepad1.start && currentGamepad1.start) {
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
        telemetry.addData("Heading", "%4.2f", df.GetHeading());
        telemetry.addData("TargetAprilTag", "%d", targetAprilTag);
        telemetry.addData("Detected AprilTag ID",  "%2d", af.DetectAprilTag(targetAprilTag) ? targetAprilTag : -1);
        telemetry.addData("AprilTag Range", "%4.2f", af.detectedTag != null ? af.detectedTag.ftcPose.range : -1);
        telemetry.addData("Distance (Sensor)", "%4.2f", df.GetDistanceFromSensorInInches(0.5, 80.0));
        telemetry.addData("Speed Factor", "%4.2f", speedFactor);
        telemetry.addData("Backdrop Row Target", "%2d", rowTarget);
        telemetry.addData("Backdrop Column", "%2d", columnTarget);

        telemetry.update();
    }
}

