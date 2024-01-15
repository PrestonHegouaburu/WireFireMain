package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Common.DrivingFunctions;
import org.firstinspires.ftc.teamcode.Common.ServoFunctions;

@TeleOp(name="Calibrate Servos", group="Test TeleOp")
public class CalibrateServos extends LinearOpMode {
    private ServoFunctions sf;
    private DrivingFunctions df;
    private int currentServoIndex;
    private Gamepad currentGamepad1, previousGamepad1;
    private void Init() {
        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

        df = new DrivingFunctions(this);
        sf = new ServoFunctions(this, df);
        if(sf.servoList.size() == 0) {
            telemetry.addLine("No Servos found on this robot -- terminating");
            telemetry.update();
            return;
        }
        currentServoIndex = 0;
    }
    @Override
    public void runOpMode() {
        Init();
        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            Servo currentServo = sf.servoList.get(currentServoIndex).s;

            if (previousGamepad1.left_trigger < 0.5 && currentGamepad1.left_trigger > 0.5)
                currentServoIndex = currentServoIndex == 0 ? sf.servoList.size()-1 : currentServoIndex - 1;
            if (previousGamepad1.right_trigger < 0.5 && currentGamepad1.right_trigger > 0.5)
                currentServoIndex = currentServoIndex == sf.servoList.size() - 1 ? 0 : currentServoIndex + 1;

            if (!previousGamepad1.left_bumper && currentGamepad1.left_bumper)
                sf.MoveServoRelative(currentServo, -0.01);
            if (!previousGamepad1.right_bumper && currentGamepad1.right_bumper)
                sf.MoveServoRelative(currentServo, 0.01);

            if(!previousGamepad1.a && currentGamepad1.a) {
                double oldPosition = currentServo.getPosition();
                currentServo.setPosition(0.0);
                sleep(500);
                currentServo.setPosition(1.0);
                sleep(1000);
                currentServo.setPosition(0.0);
                sleep(1000);
                currentServo.setPosition(oldPosition);
            }

            if(!previousGamepad1.y && currentGamepad1.y) {
                sf.servoList.get(currentServoIndex).rangeStart = 0.0;
                sf.servoList.get(currentServoIndex).rangeEnd = 1.0;
                currentServo.scaleRange(0.0, 1.0);
            }
            if(!previousGamepad1.x && currentGamepad1.x) {
                sf.servoList.get(currentServoIndex).rangeStart = currentServo.getPosition();
            }
            if(!previousGamepad1.b && currentGamepad1.b) {
                sf.servoList.get(currentServoIndex).rangeEnd = currentServo.getPosition();
                currentServo.scaleRange(sf.servoList.get(currentServoIndex).rangeStart, sf.servoList.get(currentServoIndex).rangeEnd);
            }

            UpdateTelemetry();
        }
    }
    private void UpdateTelemetry() {
        telemetry.addData("Total Servos Found", "%d", sf.servoList.size());
        telemetry.addData("Current Servo", sf.servoList.get(currentServoIndex).name);
        telemetry.addData("Current Position (relative to range)", "%4.2f", sf.servoList.get(currentServoIndex).s.getPosition());
        telemetry.addData("Range start (absolute)", "%4.2f", sf.servoList.get(currentServoIndex).rangeStart);
        telemetry.addData("Range end (absolute)", "%4.2f", sf.servoList.get(currentServoIndex).rangeEnd);

        telemetry.addLine("==============================");
        telemetry.addLine("To select next/previous servo --> Right/Left Trigger");
        telemetry.addLine("To move forward/backward by 0.01 --> Right/Left Bumper");
        telemetry.addLine("To move quickly from 0 to 1 and back (in current range) --> Press A");
        telemetry.addLine("To reset range and move to the middle --> Press Y");
        telemetry.addLine("To set the start of new range (after reset) --> Press X");
        telemetry.addLine("To set the end of new range (after reset) and scale again --> Press B");
        telemetry.update();
    }

}

