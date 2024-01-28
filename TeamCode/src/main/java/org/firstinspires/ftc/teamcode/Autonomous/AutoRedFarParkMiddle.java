package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;
@Autonomous(name="Auto - Red - Far - Park Middle", group="Auto - Red", preselectTeleOp="Game TeleOp - Red")
public class AutoRedFarParkMiddle extends AutonomousOpenCV {
    @Override
    public void runOpMode() {
        isRed = true;
        isNear = false;
        cornerPark = false;
        super.runOpMode();
    }
}

