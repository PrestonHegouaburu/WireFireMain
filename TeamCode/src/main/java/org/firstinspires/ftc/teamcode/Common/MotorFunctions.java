package org.firstinspires.ftc.teamcode.Common;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

public class MotorFunctions {
    private DcMotor intakeMotor = null;
    private DcMotor leftLiniarSlide = null;
    private DcMotor rightLiniarSlide = null;
    private LinearOpMode lom = null;

    public MotorFunctions(LinearOpMode l)
    {
        lom = l;
        Initialize();
    }
    public void Initialize(){
        intakeMotor  = lom.hardwareMap.get(DcMotor.class, "intakeMotor");
        leftLiniarSlide = lom.hardwareMap.get(DcMotor.class, "leftLiniarSlide");
        rightLiniarSlide = lom.hardwareMap.get(DcMotor.class, "rightLiniarSlide");
    }
    public void intake(double speed){
        intakeMotor.setPower(speed);
    }
    public void Moveslide(double speed){
        leftLiniarSlide.setPower(speed);
        rightLiniarSlide.setPower(-speed);


    }
    public void moveSlideDistance(double speed, int distance){
        leftLiniarSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiniarSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiniarSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiniarSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLiniarSlide.setTargetPosition(distance);
        rightLiniarSlide.setTargetPosition(distance);
        Moveslide(speed);
        while (leftLiniarSlide.isBusy() && rightLiniarSlide.isBusy()){

        }
        Moveslide(0);
    }
}