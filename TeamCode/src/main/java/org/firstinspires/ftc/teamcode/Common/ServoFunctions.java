package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public class ServoFunctions {
    private final LinearOpMode lom;
    private final DrivingFunctions df;
    private final MotorFunctions mf;
    private Servo pixelReleaseServo = null;
    private Servo planeLaunchServo = null;
    private Servo planeStage2LaunchServo = null;
    private static final double PixelReleaseInitialPosition = 0.12;
    static final int     SERVO_SMOOTH_MOVE_STEPS   = 30;     // Larger is smoother, but potentially slower

    static public class ServoInfo
    {
        public String name;
        public Servo s;
        public double rangeStart, rangeEnd;
        public ServoInfo(Servo s, String name, double rangeStart, double rangeEnd)
        {
            this.name = name;
            this.s = s;
            this.rangeStart = rangeStart;
            this.rangeEnd = rangeEnd;
        }
    }
    public List<ServoInfo> servoList;
    public ServoFunctions(LinearOpMode l, DrivingFunctions df, MotorFunctions mf)
    {
        lom = l;
        this.df = df;
        this.mf = mf;
        Initialize();
    }
    private void Initialize()
    {
        double rangeStart, rangeEnd;
        String servoName;
        servoList = new ArrayList<>();

        try {
            servoName = "PixelReleaseServo";
            pixelReleaseServo = lom.hardwareMap.get(Servo .class, servoName);
            if(df.isSlideRobot()) {
                rangeStart= 0.03;
                rangeEnd = 0.75;
            }
            else {
                rangeStart= 0.4;
                rangeEnd = 0.85;
            }
            pixelReleaseServo.scaleRange(rangeStart, rangeEnd);
            servoList.add(new ServoInfo(pixelReleaseServo, servoName, rangeStart, rangeEnd));
            pixelReleaseServo.setPosition(PixelReleaseInitialPosition);
        }
        catch(Exception e) {}

        try {
            servoName = "PlaneLaunchServo";
            planeLaunchServo = lom.hardwareMap.get(Servo .class, servoName);

            rangeStart = 0.17;
            rangeEnd = 0.48;
            servoList.add(new ServoInfo(planeLaunchServo, servoName, rangeStart, rangeEnd));
            planeLaunchServo.scaleRange(rangeStart, rangeEnd);
            planeLaunchServo.setPosition(1.0);
        }
        catch(Exception e) {}
        try {
            servoName = "PlaneLaunch2Servo";
            planeStage2LaunchServo = lom.hardwareMap.get(Servo .class, servoName);
            rangeStart = 0;
            rangeEnd = 1;
            servoList.add(new ServoInfo(planeStage2LaunchServo, servoName, rangeStart, rangeEnd));
            planeStage2LaunchServo.scaleRange(rangeStart, rangeEnd);
            planeStage2LaunchServo.setPosition(1.0);
        }
        catch(Exception e) {}
    }
    public void LaunchPlane()
    {
        if(planeLaunchServo == null)
            return;
        if (planeStage2LaunchServo != null) {
            planeStage2LaunchServo.setPosition(0);
            lom.sleep(100);
        }
        planeLaunchServo.setPosition(0);
        lom.sleep(500);
        planeLaunchServo.setPosition(1.0);
            lom.sleep(500);
        if (planeStage2LaunchServo != null)
            planeStage2LaunchServo.setPosition(1);
    }
    public double IdealDistanceFromBackdropToDeliver(int targetRow) {
        return 3.0;
    }
    public void PutPixelOnBackDrop(int targetRow)
    {
        // If a pixel falls on the ramp while the arm is up doing a deliver, the drivers need to trigger the emergency
        // "scoop" move, where the arm comes under the ramp and picks up the pixels
        boolean triggerScoopMove;
        df.MoveRobot(0, 0, 0, 0);
        MoveServoSmoothly(pixelReleaseServo, 0.35, 300);
        mf.MoveSlidesToRowTargetSync(0.5, targetRow);
        triggerScoopMove = lom.gamepad1.a || lom.gamepad2.a;
        MoveServoSmoothly(pixelReleaseServo, 0.98, 200);
        triggerScoopMove = lom.gamepad1.a || lom.gamepad2.a || triggerScoopMove;
        lom.sleep(500);
        triggerScoopMove = lom.gamepad1.a || lom.gamepad2.a || triggerScoopMove;
        if(!triggerScoopMove)
            MoveServoSmoothly(pixelReleaseServo, PixelReleaseInitialPosition, 600);
        else {
            TriggerScoopArmMove();
            return;
        }
        triggerScoopMove = lom.gamepad1.a || lom.gamepad2.a;
        if(!triggerScoopMove)
            mf.MoveSlidesToRowTargetSync(0.5, 0);
        else
            TriggerScoopArmMove();
    }

    private void TriggerScoopArmMove()
    {
        double slidesDownSpeed = 0.6;
        df.DriveStraight(0.5, -7, 0.0, false);
        mf.MoveSlidesToRowTargetSync(0.5, 6);
        pixelReleaseServo.setPosition(0.0);
        lom.sleep(550);
        mf.MoveSlidesToRowTargetSync(slidesDownSpeed, 3);
        pixelReleaseServo.setPosition(0.016);
        mf.MoveSlidesToRowTargetSync(slidesDownSpeed, 2);
        pixelReleaseServo.setPosition(0.05);
        mf.MoveSlidesToRowTargetSync(slidesDownSpeed, 1.5);
        pixelReleaseServo.setPosition(0.08);
        mf.MoveSlidesToRowTargetSync(slidesDownSpeed, 1);
        pixelReleaseServo.setPosition(0.096);
        mf.MoveSlidesToRowTargetSync(slidesDownSpeed, 0.5);
        pixelReleaseServo.setPosition(0.12);
        mf.MoveSlidesToRowTargetSync(slidesDownSpeed, 0);
        pixelReleaseServo.setPosition(PixelReleaseInitialPosition);
    }
    public void MoveServoRelative(Servo s, double delta)
    {
        s.setPosition(s.getPosition()+delta);
    }
    private void MoveServoSmoothly(Servo s, double endPosition, int timeInMilliseconds)
    {
        if (s == null)
            return;

        double stepSize = (endPosition - s.getPosition()) / SERVO_SMOOTH_MOVE_STEPS;
        long sleepTime = timeInMilliseconds / SERVO_SMOOTH_MOVE_STEPS;
        double position = s.getPosition();

        for (int i=0; i < SERVO_SMOOTH_MOVE_STEPS; i++)
        {
            position += stepSize;
            s.setPosition(position);
            lom.sleep(sleepTime);
        }
        s.setPosition(endPosition);
    }
}
