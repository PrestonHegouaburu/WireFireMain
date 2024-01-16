package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Test TeleOp", group="Test TeleOp")
public class TestModeTeleOp extends WireFireTeleOp {
    @Override
    public void runOpMode() {
        IsTestMode = true;
        super.runOpMode();
    }
    @Override
    protected void UpdateTelemetry() {
        telemetry.addData("Driving Direction", df.isRobotDrivingForward() ? "Forward" : "Backward");
        telemetry.addData("Speed Factor", "%4.2f", speedFactor);
        telemetry.addData("IntakeSpeed", "%4.2f", intakeSpeed);
        telemetry.addData("Left Slide Position", "%4.2f", mf.GetLeftSlidePosition());
        telemetry.addData("Right Slide Position", "%4.2f", mf.GetRightSlidePosition());
        telemetry.addData("Bot Heading", "%4.2f", df.GetHeading());
        telemetry.update();
    }
    @Override
    protected void ProcessTestCommands() {
        if (!IsTestMode)
            return;
        if (currentGamepad2.x)
            mf.SetIntakePower(intakeSpeed);
        if (currentGamepad2.b)
            mf.SetIntakePower(-intakeSpeed);
        if (!currentGamepad2.x && !currentGamepad2.b)
            mf.SetIntakePower(0);

        if (currentGamepad2.y && !previousGamepad2.y && intakeSpeed <= 0.9)
            intakeSpeed += 0.05;
        if (currentGamepad2.a && !previousGamepad2.a && intakeSpeed >= 0.1)
            intakeSpeed -= 0.05;

        if (currentGamepad2.left_bumper)
            mf.MoveSlides(-0.5);
        if (currentGamepad2.right_bumper)
            mf.MoveSlides(0.5);
        if (!currentGamepad2.left_bumper && !currentGamepad2.right_bumper)
            mf.MoveSlides(0);
    }
}