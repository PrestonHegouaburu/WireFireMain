package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Demonstration TeleOp", group="Test TeleOp")

public class DemonstrationTeleOp extends WireFireTeleOp{

    public void runOpMode() {
        isRedTeam = true;
        speedFactorMin = 0.25;
        speedFactorMax = 0.5;
        super.runOpMode();
    }
}
