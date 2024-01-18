package org.firstinspires.ftc.teamcode.Common;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorFunctions {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor intakeMotor = null;
    private DcMotor leftLinearSlide = null;
    private DcMotor rightLinearSlide = null;
    private LinearOpMode lom = null;
    private static final int MAX_DISTANCE_SLIDES = 2300;
    private static final int TICKS_PER_ROW = 200;
    private static final int FIRST_ROW_SLIDE_POSITION = 150;
    public static final int MAX_ROWS_OF_PIXELS = 10; // How high can the robot deliver pixels

    public MotorFunctions(LinearOpMode l) {
        lom = l;
        Initialize();
    }
    public void Initialize() {
        try {
            intakeMotor = lom.hardwareMap.get(DcMotor.class, "intakeMotor");
            leftLinearSlide = lom.hardwareMap.get(DcMotor.class, "leftLinearSlide");
            rightLinearSlide = lom.hardwareMap.get(DcMotor.class, "rightLinearSlide");
        }
        catch (Exception e)
        {
            return; // It means that this robot does not have those motors configured
        }

        leftLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void SetIntakePower(double power) {
        if (intakeMotor ==  null)
            return;
        intakeMotor.setPower(power);
    }
    public double GetLeftSlidePosition() {
        if(leftLinearSlide == null)
            return 0.0;
        return leftLinearSlide.getCurrentPosition();
    }
    public double GetRightSlidePosition() {
        if(rightLinearSlide == null)
            return 0.0;
        return rightLinearSlide.getCurrentPosition();
    }

    public void MoveSlides(double speed) {
        if (leftLinearSlide ==  null || rightLinearSlide == null)
            return;
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MoveSlidesMotor(speed);
    }
    private void MoveSlidesMotor(double speed) {
        if (speed < 0 && (leftLinearSlide.getCurrentPosition() <= 0 || rightLinearSlide.getCurrentPosition() <= 0))
            speed = 0;
        if (speed > 0 && (leftLinearSlide.getCurrentPosition() >= MAX_DISTANCE_SLIDES || rightLinearSlide.getCurrentPosition() >= MAX_DISTANCE_SLIDES))
            speed = 0;
        leftLinearSlide.setPower(speed);
        rightLinearSlide.setPower(speed);
    }
    public void MoveSlidesToRowTarget(double speed, int rowTarget) {
        double startTime = runtime.milliseconds();
        StartMovingSlidesToRowTarget(speed, rowTarget);
        while(!AreSlidesDoneMovingToTarget())
            if(runtime.milliseconds() - startTime > 2000) //waits until the slides are done moving or 2 seconds
                break;
    }
    public void StartMovingSlidesToRowTarget(double speed, int rowTarget) {
        StartMovingSlidesToPosition(speed, GetSlidesTargetPosition(rowTarget));
    }
    public boolean AreSlidesDoneMovingToTarget() {
        if(leftLinearSlide == null || rightLinearSlide == null)
            return true;
        boolean doneMoving = !lom.opModeIsActive() || !leftLinearSlide.isBusy() || !rightLinearSlide.isBusy();
        if(doneMoving)
            MoveSlides(0.0);
        return doneMoving;
    }
    private int GetSlidesTargetPosition(int targetRow) {
        if (targetRow == 0)
            return 0;
        return FIRST_ROW_SLIDE_POSITION + TICKS_PER_ROW * (targetRow-1);
    }
    private void StartMovingSlidesToPosition(double speed, int position) {
        if (leftLinearSlide ==  null || rightLinearSlide == null)
            return;
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setTargetPosition(position);
        rightLinearSlide.setTargetPosition(position);
        MoveSlidesMotor(speed);
    }
}