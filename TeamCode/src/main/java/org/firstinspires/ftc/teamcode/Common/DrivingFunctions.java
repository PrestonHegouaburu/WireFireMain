package org.firstinspires.ftc.teamcode.Common;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;


public class DrivingFunctions {
    private boolean isRobotA = false;
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DistanceSensor distanceSensor = null;
    private boolean drivingForward = true;
    private IMU imu = null;
    private AHRS navx_device;
    private LinearOpMode lom = null;
    private double headingError = 0.0;
    private final ElapsedTime runtime = new ElapsedTime();
    static final int TURN_TIMEOUT_MILLISECONDS = 3000;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // Our motors are these: https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // These are our Mecanum wheels (96mm) - https://www.gobilda.com/96mm-mecanum-wheel-set-70a-durometer-bearing-supported-rollers/
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     HEADING_THRESHOLD       = 2.0 ;    // How close must the heading get to the target before moving to next step.
    static final double     P_TURN_GAIN            = 0.04;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
    public DrivingFunctions(LinearOpMode l) {
        lom = l;
        Initialize();
    }
    private void DetermineWhatRobotThisIs() {
        // In our Wire Fire Robot B, we named the front-left motor "b-frontleft". If that's found, then this is RobotB.
        // If this throws an exception, then this is Robot A
        try {
            leftFrontDrive = lom.hardwareMap.get(DcMotor.class, "b-frontleft");
            isRobotA = false; // If this line is executed, the above call didn't fail, so this is RobotB
        }
        catch (Exception e) {
            isRobotA = true;
        }
    }
    public boolean isSlideRobot()
    {
        return !isRobotA;
    }
    public boolean isRobotDrivingForward() {return drivingForward;}
    private void Initialize() {
        DetermineWhatRobotThisIs();
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        if(!isSlideRobot())
            leftFrontDrive  = lom.hardwareMap.get(DcMotor.class, "frontleft");
        leftBackDrive  = lom.hardwareMap.get(DcMotor.class, "backleft");
        rightFrontDrive = lom.hardwareMap.get(DcMotor.class, "frontright");
        rightBackDrive = lom.hardwareMap.get(DcMotor.class, "backright");
        try {
            distanceSensor = lom.hardwareMap.get(DistanceSensor.class, "distanceSensor");
        }
        catch(Exception e) {
        }

        if(isSlideRobot()) {
            navx_device = AHRS.getInstance(lom.hardwareMap.get(NavxMicroNavigationSensor.class, "navx"), AHRS.DeviceDataType.kProcessedData);
        }
        else {
            imu = lom.hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    isSlideRobot() ? RevHubOrientationOnRobot.LogoFacingDirection.LEFT : RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    isSlideRobot() ? RevHubOrientationOnRobot.UsbFacingDirection.FORWARD : RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);
        }
        SetDirectionForward();
        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        StopMotors();
        ResetYaw();
    }

    public void SetDirectionForward() {
        drivingForward = true;
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(isSlideRobot() ? DcMotor.Direction.REVERSE: DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }
    public void SetDirectionBackward() {
        drivingForward = false;
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(isSlideRobot() ? DcMotor.Direction.FORWARD: DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }
    public double GetDistanceFromSensorInInches(double minExpectedDistance, double maxExpectedDistance) {
        if (distanceSensor == null)
            return 0.0;
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        distance = distance < minExpectedDistance || distance > maxExpectedDistance ? 0.0 : distance;
        return distance;
    }
    public void ResetYaw()
    {
        if(isSlideRobot())
            navx_device.zeroYaw();
        else
            imu.resetYaw();
    }

    /**
     *  Drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void DriveStraight(double maxDriveSpeed, double distance, double heading, boolean strafe) {
        if(distance == 0.0)
            return;
        CalculateHeadingError(heading);
        if (Math.abs(this.headingError) > 4.0)
            TurnToHeading(maxDriveSpeed, heading);
        // Determine new target position, and pass to motor controller
        int moveCounts = (int)(distance * COUNTS_PER_INCH);

        // Set Target FIRST, then turn on RUN_TO_POSITION
        leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + moveCounts);
        rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition() + moveCounts);
        if (!strafe) {
            rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() + moveCounts);
            leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() + moveCounts);
        } else {
            rightFrontDrive.setTargetPosition(rightFrontDrive.getCurrentPosition() - moveCounts);
            leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition() - moveCounts);
        }
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the required driving speed  (must be positive for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop
        maxDriveSpeed = Math.abs(maxDriveSpeed);
        if (!strafe) {
            MoveRobot(0, maxDriveSpeed, 0, 1.0);
        } else{
            MoveRobot(maxDriveSpeed, 0, 0, 1.0);
        }
        // keep looping while we are still active, and BOTH motors are running.
        while (lom.opModeIsActive() &&
                (leftFrontDrive.isBusy() && rightFrontDrive.isBusy() && leftBackDrive.isBusy()
                        && rightBackDrive.isBusy())) {
            // Determine required steering to keep on heading
            double turnSpeed = GetSteeringCorrection(heading, P_DRIVE_GAIN);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                turnSpeed *= -1.0;

            // Apply the turning correction to the current driving speed.
            if (!strafe) {
                MoveRobot(0, maxDriveSpeed, -turnSpeed, 1.0);
            } else {
                MoveRobot(maxDriveSpeed, 0, -turnSpeed, 1.0);
            }
        }
        StopMotors();
    }
    private void StopMotors() {
        // Stop all motion & Turn off RUN_TO_POSITION
        MoveRobot(0, 0, 0, 1.0);
        // Set the encoders for closed loop speed control
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     *  Spin on the central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the OpMode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void TurnToHeading(double maxTurnSpeed, double heading) {

        double startTime = runtime.milliseconds();
        // Run getSteeringCorrection() once to pre-calculate the current error
        CalculateHeadingError(heading);
        int timeout = Math.abs(this.headingError) < 90 ? TURN_TIMEOUT_MILLISECONDS / 2 : TURN_TIMEOUT_MILLISECONDS;

        // keep looping while we are still active, and not on heading.
        while ((runtime.milliseconds() - startTime) < timeout &&
                lom.opModeIsActive() &&
                ((Math.abs(this.headingError) > HEADING_THRESHOLD) || Math.abs(GetRotatingSpeed()) > 0.05)) {
            // Determine required steering to keep on heading
            double turnSpeed = GetSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            MoveRobot(0, 0, -turnSpeed, 1.0);
        }
        // Stop all motion;
        StopMotors();
    }

    /* Assumes that the robot is in a position to see the desired tag, and fully squared parallel to the backdrop. If it can't see the desired tag, it returns false.
    After successfully driving to the desired tag (aligning perfectly so it is facing it directly at the desired distance, it returns true
     */
    public boolean DriveToAprilTagAutonomous(AprilTagsFunctions atf, double desiredHeading, int desiredTag,
                                   double desiredDistanceFromTagInches, double speedFactor) {
        // Strafes left or right to align to target
        if(!atf.DetectAprilTag(desiredTag))
            return false;
        DriveStraight(0.7 * speedFactor, atf.detectedTag.ftcPose.x, desiredHeading, true);

        // Uses AprilTag to get to 10 inches from target. If it's already at less than 10, it doesn't move
        if (!atf.DetectAprilTag(desiredTag))
            return false;
        double distance = atf.detectedTag.ftcPose.range - desiredDistanceFromTagInches;
        if (distance > 10) {
            DriveStraight(speedFactor, distance - 10, desiredHeading, false);
            distance = 10;
        }

        // Strafes left or right to align to target once again
        if(atf.DetectAprilTag(desiredTag))
            DriveStraight(0.7 * speedFactor, atf.detectedTag.ftcPose.x, desiredHeading, true);

        // Uses the distance sensor to get to the final position
        // If the sensor returns more than 15 inches or less than 2 inches, we assume the sensor is wrong
        // If the sensor is not working, it uses the latest distance estimation from the AprilTag
        double sensorDistance = GetDistanceFromSensorInInches(2.0, 15.0);
        distance = sensorDistance > 0.0 ? sensorDistance : distance;
        DriveStraight(0.5 * speedFactor, distance - desiredDistanceFromTagInches, desiredHeading, false);

        return true;
    }

    public boolean DriveToAprilTagTeleop(AprilTagsFunctions atf, double desiredHeading, int desiredTag, double horizontalShiftFromTag,
                                             double desiredDistanceFromTagInches, double speedFactor) {

        // Strafes to align with the April tag
        if(!atf.DetectAprilTag(desiredTag))
            return false;
        DriveStraight(speedFactor, atf.detectedTag.ftcPose.x, GetHeading(), true);

        // Gets to 15" from AprilTag
        if(!atf.DetectAprilTag(desiredTag))
            return false;
        double horizontalDistance = atf.detectedTag.ftcPose.range * Math.sin(Math.toRadians(atf.detectedTag.ftcPose.yaw));
        horizontalDistance += atf.detectedTag.ftcPose.x;

        if (atf.detectedTag.ftcPose.range > 15)
            DriveStraight(speedFactor, atf.detectedTag.ftcPose.range-15, GetHeading(), false);
        if(atf.DetectAprilTag(desiredTag)) {
            horizontalDistance = atf.detectedTag.ftcPose.range * Math.sin(Math.toRadians(atf.detectedTag.ftcPose.yaw));
            horizontalDistance += atf.detectedTag.ftcPose.x;
        }
        // Faces the backdrop
        TurnToHeading(speedFactor, desiredHeading);
        // Strafes left or right to align to target
        DriveStraight(0.7 * speedFactor, horizontalDistance+horizontalShiftFromTag, desiredHeading, true);

        double distance = 10.0;
        // Uses the distance sensor to get to the final position
        // If the sensor returns more than 18 inches or less than 2 inches, we assume the sensor is wrong
        // If the sensor is not working, it uses the latest distance estimation from the AprilTag
        double sensorDistance = GetDistanceFromSensorInInches(2.0, 18.0);
        distance = sensorDistance > 0.0 ? sensorDistance : distance;
        DriveStraight(0.5 * speedFactor, distance - desiredDistanceFromTagInches, desiredHeading, false);

        return true;
    }

    // **********  LOW Level driving functions.  ********************
    /**
     * Use a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    private double GetSteeringCorrection(double desiredHeading, double proportionalGain) {
        CalculateHeadingError(desiredHeading);
        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(this.headingError * proportionalGain, -1, 1);
    }

    private void CalculateHeadingError(double desiredHeading) {
        this.headingError = desiredHeading - GetHeading();
        // Normalize the error to be within +/- 180 degrees
        while (this.headingError > 180)  this.headingError -= 360;
        while (this.headingError <= -180) this.headingError += 360;
    }

    public void MoveRobot(double x, double y, double yaw, double speedFactor) {
        // If yaw is positive, it turns to the right
        double max;
        double leftFrontPower  = y + x + yaw;
        double rightFrontPower = y - x - yaw;
        double leftBackPower   = y - x + yaw;
        double rightBackPower  = y + x - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower * speedFactor);
        rightFrontDrive.setPower(rightFrontPower * speedFactor);
        leftBackDrive.setPower(leftBackPower * speedFactor);
        rightBackDrive.setPower(rightBackPower * speedFactor);
    }
    public double GetHeading() {
        double heading;
        if(isSlideRobot())
            heading = navx_device.getYaw();
        else
            heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if(isRobotDrivingForward())
            heading = heading * -1;
        return heading;
    }
    public double GetRotatingSpeed() {
        double rotatingSpeed;
        if(isSlideRobot())
            rotatingSpeed = -navx_device.getWorldLinearAccelZ();
        else
            rotatingSpeed = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate / 100.0;


        if(isRobotDrivingForward())
            rotatingSpeed = rotatingSpeed * -1;
        return rotatingSpeed;
    }

    public void TestWheelsEncoders() {
        TestWheelEncoder("Front Left", leftFrontDrive, true);
        TestWheelEncoder("Front Left", leftFrontDrive, false);
        TestWheelEncoder("Front Right", rightFrontDrive, true);
        TestWheelEncoder("Front Right", rightFrontDrive, false);
        TestWheelEncoder("Back Left", leftBackDrive, true);
        TestWheelEncoder("Back Left", leftBackDrive, false);
        TestWheelEncoder("Back Right", rightBackDrive, true);
        TestWheelEncoder("Back Right", rightBackDrive, false);
    }
    private void TestWheelEncoder(String wheelName, DcMotor wheel, boolean forward)
    {
        String movingDirection;
        movingDirection = forward ? "Moving Forward..." : "Moving Backward...";
        lom.telemetry.addData("Testing Wheel", wheelName);
        lom.telemetry.addLine(movingDirection);
        wheel.setPower(forward ? 0.1 : -0.1);
        for(int i = 0; i < 10; i++) {
            lom.telemetry.addData("Testing Wheel", wheelName);
            lom.telemetry.addLine(movingDirection);
            lom.telemetry.addData("Wheel position", "%d", wheel.getCurrentPosition());
            lom.telemetry.update();
            lom.sleep(300);
        }
        wheel.setPower(0);
    }
}
