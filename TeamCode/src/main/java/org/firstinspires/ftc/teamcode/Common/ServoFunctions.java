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
    private Servo fingerServoTop = null;
    private Servo fingerServoBottom = null;
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
    static final int     SERVO_SMOOTH_MOVE_STEPS   = 30;     // Larger is smoother, but potentially slower
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
                rangeStart= 0.11;
                rangeEnd = 0.76;
            }
            else {
                rangeStart= 0.4;
                rangeEnd = 0.85;
            }
            pixelReleaseServo.scaleRange(rangeStart, rangeEnd);
            servoList.add(new ServoInfo(pixelReleaseServo, servoName, rangeStart, rangeEnd));
            pixelReleaseServo.setPosition(0.0);
        }
        catch(Exception e) {}

        try {
            servoName = "PlaneLaunchServo";
            planeLaunchServo = lom.hardwareMap.get(Servo .class, servoName);
            rangeStart = 0.0;
            rangeEnd = 1.0;
            servoList.add(new ServoInfo(planeLaunchServo, servoName, rangeStart, rangeEnd));
            planeLaunchServo.scaleRange(rangeStart, rangeEnd);
            planeLaunchServo.setPosition(0.0);
        }
        catch(Exception e) {}

        try {
            servoName = "FingerServoBottom";
            fingerServoBottom = lom.hardwareMap.get(Servo .class, servoName);
            rangeStart = 0.0;
            rangeEnd = 1.0;
            servoList.add(new ServoInfo(fingerServoBottom, servoName, rangeStart, rangeEnd));
            fingerServoBottom.scaleRange(rangeStart, rangeEnd);
            fingerServoBottom.setPosition(0.0);
        }
        catch(Exception e) {}

        try {
            servoName = "FingerServoTop";
            fingerServoTop = lom.hardwareMap.get(Servo .class, servoName);
            rangeStart = 0.0;
            rangeEnd = 1.0;
            servoList.add(new ServoInfo(fingerServoTop, servoName, rangeStart, rangeEnd));
            fingerServoTop.scaleRange(rangeStart, rangeEnd);
            fingerServoTop.setPosition(0.0);
        }
        catch(Exception e) {}
    }
    public void LaunchPlane()
    {
        if(planeLaunchServo == null)
            return;
        planeLaunchServo.setPosition(1.0);
        lom.sleep(200);
        planeLaunchServo.setPosition(0.0);
    }
    public double IdealDistanceFromBackdropToDeliver(int targetRow) {
        return 3.0;
    }
    public void PutPixelOnBackDrop(int targetRow)
    {
        df.MoveRobot(0, 0, 0, 0);
        MoveServoSmoothly(pixelReleaseServo, 0.3, 300);
        mf.MoveSlidesToRowTargetSync(0.5, targetRow);
        MoveServoSmoothly(pixelReleaseServo, 1.0, 200);
        lom.sleep(500);
        MoveServoSmoothly(pixelReleaseServo, 0.0, 600);
        mf.MoveSlidesToRowTargetSync(0.5, 0);
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
