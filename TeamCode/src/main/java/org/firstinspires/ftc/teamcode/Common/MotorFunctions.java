package org.firstinspires.ftc.teamcode.Common;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class MotorFunctions {
    private DcMotor intakeMotor = null;
    private DcMotor leftLinearSlide = null;
    private DcMotor rightLinearSlide = null;
    private LinearOpMode lom = null;
    private static int MAX_DISTANCE_SLIDES = 1400;

    public MotorFunctions(LinearOpMode l)
    {
        lom = l;
        Initialize();
    }
    public void Initialize(){
        intakeMotor  = lom.hardwareMap.get(DcMotor.class, "intakeMotor");
        leftLinearSlide = lom.hardwareMap.get(DcMotor.class, "leftLinearSlide");
        rightLinearSlide = lom.hardwareMap.get(DcMotor.class, "rightLinearSlide");

        leftLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void intake(double speed){
        intakeMotor.setPower(speed);
    }
    public void MoveSlide(double speed) {
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (speed < 0 && (leftLinearSlide.getCurrentPosition() <= 0 || rightLinearSlide.getCurrentPosition() <= 0))
            speed = 0;
        if (speed > 0 && (leftLinearSlide.getCurrentPosition() >= MAX_DISTANCE_SLIDES || rightLinearSlide.getCurrentPosition() >= MAX_DISTANCE_SLIDES))
            speed = 0;
        leftLinearSlide.setPower(speed);
        rightLinearSlide.setPower(speed);
    }

    public void moveSlideDistance(double speed, int distance)
    {
        leftLinearSlide.setTargetPosition(distance);
        rightLinearSlide.setTargetPosition(distance);
        MoveSlide(speed);
        while (leftLinearSlide.isBusy() && rightLinearSlide.isBusy()){

        }
        MoveSlide(0);
    }

    public void PrintMotorPositions(double speed)
    {
        lom.telemetry.addData("Speed: ",  "%3.2f", speed);
        lom.telemetry.addData("Left Slide Position: ",  "%7d", leftLinearSlide.getCurrentPosition());
        lom.telemetry.addData("Right Front Position: ",  "%7d", rightLinearSlide.getCurrentPosition());
        lom.telemetry.update();
    }

}