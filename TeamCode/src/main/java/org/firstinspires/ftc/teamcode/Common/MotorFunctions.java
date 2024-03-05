package org.firstinspires.ftc.teamcode.Common;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOp.WireFireTeleOp;

public class MotorFunctions {
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor intakeMotor = null;
    private DcMotor leftLinearSlide = null;
    private DcMotor rightLinearSlide = null;
    private final LinearOpMode lom;
    private final WireFireTeleOp wireFireTeleOp;
    private static final int MAX_DISTANCE_SLIDES = 2300;
    private static final int TICKS_PER_ROW = 200;
    private static final int FIRST_ROW_SLIDE_POSITION = 150;
    public static final int MAX_ROWS_OF_PIXELS = 10; // How high can the robot deliver pixels
    public MotorFunctions(LinearOpMode l, WireFireTeleOp wireFireTeleOp) {
        lom = l;
        this.wireFireTeleOp = wireFireTeleOp;
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
    public void MoveSlidesToRowTargetSync(double speed, double rowTarget ,boolean keepMovingRobot) {
        if (leftLinearSlide ==  null || rightLinearSlide == null)
            return;
        double startTime = runtime.milliseconds();
        int position = GetSlidesTargetPosition(rowTarget);
        leftLinearSlide.setTargetPosition(position);
        rightLinearSlide.setTargetPosition(position);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MoveSlides(speed);

        while(lom.opModeIsActive() && leftLinearSlide.isBusy() && rightLinearSlide.isBusy()) {
            if(keepMovingRobot && wireFireTeleOp != null)
                wireFireTeleOp.ActiveSleep(1);
            if (runtime.milliseconds() - startTime > 2000) //waits until the slides are done moving or 2 seconds
                break;
        }

        leftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MoveSlides(0.0);
    }
    private int GetSlidesTargetPosition(double rowTarget) {
        if (rowTarget == 0)
            return 0;
        return (int) (FIRST_ROW_SLIDE_POSITION + TICKS_PER_ROW * (rowTarget-1));
    }
}