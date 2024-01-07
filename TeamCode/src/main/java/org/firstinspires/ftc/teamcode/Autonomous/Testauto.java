package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.DrivingFunctions;
import org.firstinspires.ftc.teamcode.Common.ServoFunctions;
import org.firstinspires.ftc.teamcode.Common.MotorFunctions;
@Autonomous(name="TestSensir", group="Test")
public class Testauto extends LinearOpMode {



    private DrivingFunctions df = null;
    @Override
    public void runOpMode()
    {
        waitForStart();
        df = new DrivingFunctions(this);
       df.goToDistance(2,0);
       telemetry.addData("Distance",df.GetDistanceFromSensorInInches());
        telemetry.update();
    }
}
