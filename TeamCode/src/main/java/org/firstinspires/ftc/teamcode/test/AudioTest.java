package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Audio Test", group = "A - Test")
public class AudioTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Press A to test audio");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) {
                telemetry.speak("Shooter ready!");
                telemetry.addLine("Shooter ready!");
            }

            if (gamepad1.bWasPressed()) {
                telemetry.speak("Target locked!");
            }

            if (gamepad1.xWasPressed()) {
                telemetry.speak("Intake jam detected!");
            }

            telemetry.update();
        }
    }
}