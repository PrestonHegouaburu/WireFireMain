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
    public void intake(float speed){
        intakeMotor.setPower(speed);
    }
    public void Moveslide(float speed, float distance){
        leftLiniarSlide.setPower(speed);
        rightLiniarSlide.setPower(speed);
    }

}