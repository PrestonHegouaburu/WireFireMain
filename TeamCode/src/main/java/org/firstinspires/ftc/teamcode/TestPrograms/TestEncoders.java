package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.DrivingFunctions;
import org.firstinspires.ftc.teamcode.Common.ServoFunctions;

@Autonomous(name="Autonomous - Test Encoders", group="Other Tests")
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
