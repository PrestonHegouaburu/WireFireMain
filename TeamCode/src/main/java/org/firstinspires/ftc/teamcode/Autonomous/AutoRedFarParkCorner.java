package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;
@Autonomous(name="Auto - Red - Far - Park Corner", group="Auto - Red", preselectTeleOp="Game TeleOp - Red")
public class AutoRedFarParkCorner extends AutonomousOpenCV {
    @Override
    public void runOpMode() {
        isRed = true;
        isNear = false;
        cornerPark = true;
        super.runOpMode();
    }
}
