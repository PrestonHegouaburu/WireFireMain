package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Common.DrivingFunctions;
@Autonomous(name="intakeTest", group="Auto Test")
public class IntakeTest extends LinearOpMode{
    private DrivingFunctions df = null;
    public void runOpMode() {
        df = new DrivingFunctions(this);
        //df.intake(100000, 0.6);
    }
}
