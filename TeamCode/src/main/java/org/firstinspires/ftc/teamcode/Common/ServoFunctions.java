package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public class ServoFunctions {
    private LinearOpMode lom = null;
    private DrivingFunctions df;
    private MotorFunctions mf;
    private Servo pixelReleaseServo = null;
    private Servo planeLaunchServo = null;
    private static final double PixelReleaseInitialPosition = 0.12;
    static final int     SERVO_SMOOTH_MOVE_STEPS   = 30;     // Larger is smoother, but potentially slower

    public class ServoInfo
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
        servoList = new ArrayList<ServoInfo>();

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
    }
    public void LaunchPlane()
    {
        if(planeLaunchServo == null)
            return;
        planeLaunchServo.setPosition(0.0);
        lom.sleep(500);
        planeLaunchServo.setPosition(1.0);
    }
    public double IdealDistanceFromBackdropToDeliver(int targetRow) {
        return 3.0;
    }
    public void PutPixelOnBackDrop(int targetRow)
    {
        // If a pixel falls on the ramp while the arm is up doing a deliver, the drivers need to trigger the emergency
        // "scoop" move, where the arm comes under the ramp and picks up the pixels
        boolean triggerScoopMove = false;
        df.MoveRobot(0, 0, 0, 0);
        MoveServoSmoothly(pixelReleaseServo, 0.35, 300);
        mf.MoveSlidesToRowTargetSync(0.5, targetRow);
        triggerScoopMove = lom.gamepad1.a || lom.gamepad2.a || triggerScoopMove;
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
        triggerScoopMove = lom.gamepad1.a || lom.gamepad2.a || triggerScoopMove;
        if(!triggerScoopMove)
            mf.MoveSlidesToRowTargetSync(0.5, 0);
        else
            TriggerScoopArmMove();
    }

    private void TriggerScoopArmMove()
    {
        df.DriveStraight(0.5, -7, 0.0, false);
        mf.MoveSlidesToRowTargetSync(0.5, 6);
        pixelReleaseServo.setPosition(0.0);
        lom.sleep(550);
        mf.MoveSlidesToRowTargetSync(0.5, 3);
        pixelReleaseServo.setPosition(0.016);
        mf.MoveSlidesToRowTargetSync(0.5, 2);
        pixelReleaseServo.setPosition(0.05);
        mf.MoveSlidesToRowTargetSync(0.5, 1);
        pixelReleaseServo.setPosition(0.08);
        mf.MoveSlidesToRowTargetSync(0.5, 0);
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
