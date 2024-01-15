package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Game TeleOp - Blue", group="Game TeleOp")
public class GameTeleOpBlue extends WireFireTeleOp {
    @Override
    public void runOpMode()
    {
        IsRedTeam = false;
        super.runOpMode();
    }
}
