package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class ServoFunctions {
    private LinearOpMode lom = null;
    private DrivingFunctions df;
    private Servo pixelReleaseServo = null;
    private Servo planeLaunchServo = null;
    private Servo fingerStraightenServo = null;
    static final int     SERVO_SMOOTH_MOVE_STEPS   = 30;     // Larger is smoother, but potentially slower
    public ServoFunctions(LinearOpMode l, DrivingFunctions df)
    {
        lom = l;
        this.df = df;
        Initialize();
    }
    private void Initialize()
    {
        try {
            pixelReleaseServo = lom.hardwareMap.get(Servo .class, "PixelReleaseServo");
           // planeLaunchServo = lom.hardwareMap.get(Servo .class, "PlaneLaunchServo");
            fingerStraightenServo = lom.hardwareMap.get(Servo .class, "FingerStraightenServo");
            if(df.isSlideRobot()) {
                pixelReleaseServo.scaleRange(0.3, 0.94);
                fingerStraightenServo.scaleRange(0, 0.4);
            }
            else {
                pixelReleaseServo.scaleRange(0.35, 0.85);
                //planeLaunchServo.scaleRange(0, 1);

            }

            pixelReleaseServo.setPosition(0.0);
        }
        catch(Exception e) {

        }

    }
    public void PutPixelInBackBoard()
    {
        MoveServoSmoothly(pixelReleaseServo, 1.0, 800);
        lom.sleep(300);
        MoveServoSmoothly(pixelReleaseServo, 0.0, 500);
    }
    public void PutPixelInBackBoardSlides()
    {
        MoveServoSmoothly(fingerStraightenServo, 1.0, 500);
        MoveServoSmoothly(pixelReleaseServo, 1.0, 800);
        lom.sleep(300);
        MoveServoSmoothly(fingerStraightenServo, 0.0, 500);
        MoveServoSmoothly(pixelReleaseServo, 0.0, 500);
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
    public void MovePixelReleaseServoRelative(double move)
    {
        if (pixelReleaseServo == null)
            return;
        MoveServoSmoothly(pixelReleaseServo, pixelReleaseServo.getPosition() + move, 100);
    }
    public double GetPixelReleaseServoPosition()
    {
        if (pixelReleaseServo == null)
            return 0.0;

        return pixelReleaseServo.getPosition();
    }

    public double GetPlaneLauncherServoPosition()
    {
        if (planeLaunchServo == null)
            return 0.0;

        return planeLaunchServo.getPosition();
    }
}
