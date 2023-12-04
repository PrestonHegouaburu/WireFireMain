package org.firstinspires.ftc.teamcode.TestPrograms;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Common.MotorFunctions;

@TeleOp(name="SlideTest", group="Auto Test")
public class SlidesTest extends LinearOpMode{
    private MotorFunctions mf = null;
    public void runOpMode() {
        mf = new MotorFunctions(this);
        waitForStart();
        double speed = 0.3;
        double intakeSpeed = 0.3;
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            if (!previousGamepad1.x && currentGamepad1.x && speed > 0.1)
                speed -= 0.1;
            if (!previousGamepad1.b && currentGamepad1.b && speed < 0.9)
                speed += 0.1;
            if (!previousGamepad1.y && currentGamepad1.y && intakeSpeed > 0.1)
                intakeSpeed -= 0.1;
            if (!previousGamepad1.a && currentGamepad1.a && intakeSpeed < 0.9)
                intakeSpeed += 0.1;
            mf.PrintMotorPositions(speed);

            if(gamepad1.right_trigger > 0.5)
                mf.intake(intakeSpeed);
            else if(gamepad1.left_trigger > 0.5)
                mf.intake(-intakeSpeed);
            else
                mf.intake(0);

            if(gamepad1.right_bumper)
                mf.MoveSlide(speed);
            else if(gamepad1.left_bumper)
                mf.MoveSlide(-speed);
            else
                mf.MoveSlide(0);


//            mf.moveSlideDistance(0.3, 100);
//            mf.moveSlideDistance(0.3, 1);
//            mf.moveSlideDistance(0.3, 200);
        }
    }

}

