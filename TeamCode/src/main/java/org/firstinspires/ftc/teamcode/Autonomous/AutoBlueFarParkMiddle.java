package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpenCV;
@Autonomous(name="Auto - Blue - Far - Park Middle", group="Auto - Blue", preselectTeleOp="Game TeleOp - Blue")
public class AutoBlueFarParkMiddle extends AutonomousOpenCV {
    @Override
    public void runOpMode() {
        isRed = false;
        isNear = false;
        cornerPark = false;
        super.runOpMode();
    }
}
