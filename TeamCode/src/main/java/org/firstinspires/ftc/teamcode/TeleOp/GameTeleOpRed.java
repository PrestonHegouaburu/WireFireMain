package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Game TeleOp - Red", group="Game TeleOp")
public class GameTeleOpRed extends WireFireTeleOp {
    @Override
    public void runOpMode() {
        isRedTeam = true;
        super.runOpMode();
    }
}
