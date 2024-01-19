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
    private int currentRow = 0;
    private int slidesMovement = 0; // 0: not moving, 1: moving up, -1: moving down

    public MotorFunctions(LinearOpMode l) {
        lom = l;
        Initialize();
    }
    private void Initialize() {
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
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        if (speed < 0 && (leftLinearSlide.getCurrentPosition() <= 0 || rightLinearSlide.getCurrentPosition() <= 0))
            speed = 0;
        if (speed > 0 && (leftLinearSlide.getCurrentPosition() >= MAX_DISTANCE_SLIDES || rightLinearSlide.getCurrentPosition() >= MAX_DISTANCE_SLIDES))
            speed = 0;
        leftLinearSlide.setPower(speed);
        rightLinearSlide.setPower(speed);
    }

    /* This function is synchronous, as it waits for the whole movement to complete before returning */
    public void MoveSlidesToRowTargetSync(double speed, int rowTarget) {
        if (leftLinearSlide ==  null || rightLinearSlide == null)
            return;
        double startTime = runtime.milliseconds();
        int position = GetSlidesTargetPosition(rowTarget);
        leftLinearSlide.setTargetPosition(position);
        rightLinearSlide.setTargetPosition(position);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MoveSlides(speed);

        while(lom.opModeIsActive() && leftLinearSlide.isBusy() && rightLinearSlide.isBusy())
            if(runtime.milliseconds() - startTime > 2000) //waits until the slides are done moving or 2 seconds
                break;

        leftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MoveSlides(0.0);
    }
    public void MoveSlidesToRowTargetAsync(double slidesSpeed, int rowTarget) {
        if(leftLinearSlide == null || rightLinearSlide == null)
            return;
        int currPos = leftLinearSlide.getCurrentPosition();
        int targetPos = GetSlidesTargetPosition(rowTarget);

        if (currentRow != rowTarget && slidesMovement == 0) {
            slidesMovement = currPos < targetPos ? 1 : -1;
            MoveSlides(slidesSpeed * slidesMovement);
        }

        if((slidesMovement == -1 && currPos < targetPos) || (slidesMovement == 1 && currPos > targetPos)) {
            MoveSlides(0);
            currentRow = rowTarget;
            slidesMovement = 0;
        }
    }
    private int GetSlidesTargetPosition(int rowTarget) {
        if (rowTarget == 0)
            return 0;
        return FIRST_ROW_SLIDE_POSITION + TICKS_PER_ROW * (rowTarget-1);
    }
    public int getRowFromPosition() {
        if(leftLinearSlide == null)
            return 0;
        if (leftLinearSlide.getCurrentPosition() < FIRST_ROW_SLIDE_POSITION)
            return 0;
        int row = (leftLinearSlide.getCurrentPosition() - FIRST_ROW_SLIDE_POSITION) / TICKS_PER_ROW + 1;
        row = row > 10 ? 10 : row;
        row = row < 0 ? 0 : row;
        return row;
    }
}