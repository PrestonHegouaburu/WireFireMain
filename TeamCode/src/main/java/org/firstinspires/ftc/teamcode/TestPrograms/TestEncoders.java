package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.DrivingFunctions;

@Autonomous(name="Autonomous - Test Encoders", group="Linear Opmode")
//@Disabled
public class TestEncoders extends LinearOpMode {
    private DrivingFunctions df;
    @Override
    public void runOpMode() {
        df = new DrivingFunctions(this);
        waitForStart();
        df.TestWheelsEncoders();
    }

}
