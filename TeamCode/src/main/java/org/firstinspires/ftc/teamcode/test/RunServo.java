package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config;

//v uncommenting this will hide it on the driver station
//@Disabled
@TeleOp(name = "Run Servo", group = "Z - Test")
public class RunServo extends LinearOpMode {

    private int selected = 0;
    private double position;
    private Servo servo;

    @Override
    public void runOpMode() {

        boolean previousSelectionInput = false;

        while (opModeInInit()) {

            boolean up = gamepad1.dpad_up || gamepad1.left_stick_y < -0.25;
            boolean down = gamepad1.dpad_down || gamepad1.left_stick_y > 0.25;

            if ((up || down) && !previousSelectionInput) {
                if (up) {
                    selected--;
                } else {
                    selected++;
                }
            }

            previousSelectionInput = up || down;

            selected = Math.max(
                    0,
                    Math.min(Config.SERVOS.length - 1, selected)
            );

            // Display menu
            telemetry.addLine("=== SELECT SERVO ===");

            for (int i = 0; i < Config.SERVOS.length; i++) {
                if (i == selected) {
                    telemetry.addLine("> " + Config.SERVOS[i]);
                } else {
                    telemetry.addLine("  " + Config.SERVOS[i]);
                }
            }

            telemetry.addLine("");
            telemetry.addLine("D-Pad ^v or Left Joystick ^v: Select");
            telemetry.addLine("Press start to run!");
            telemetry.addLine("Press B / Circle to cancel.");

            telemetry.update();

            if (gamepad1.b) {
                return;
            }
        }

        servo = hardwareMap.get(
                Servo.class,
                Config.SERVOS[selected]
        );

        position = servo.getPosition();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {

            // Stick up = increase position
            // Stick down = decrease position
            position -= gamepad1.left_stick_y * 0.01;

            // Keep position between 0 and 1
            position = Math.max(0, Math.min(1, position));

            servo.setPosition(position);

            telemetry.addLine("Use left stick to control servo position");
            telemetry.addData("Servo", Config.SERVOS[selected]);
            telemetry.addData("Position", position);
            telemetry.update();
        }
    }
}