package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Common.DrivingFunctions;

@Autonomous(name="Autonomous - Test Encoders", group="Other Tests")
//@Disabled
public class TestEncoders extends LinearOpMode {
    @Override
    public void runOpMode() {
        DrivingFunctions df = new DrivingFunctions(this);
        waitForStart();
        df.TestWheelsEncoders();
    }

}
