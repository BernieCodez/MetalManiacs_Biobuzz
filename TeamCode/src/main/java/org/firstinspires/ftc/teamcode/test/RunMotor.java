package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Config;

//v uncommenting this will hide it on the driver station
//@Disabled
@TeleOp(name = "Run Motor", group = "A - Test")
public class RunMotor extends LinearOpMode {

    private int selected = 0;

    private double speed = 1;
    private boolean previousUp;
    private boolean previousDown;
    DcMotor motor;

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
        motor = hardwareMap.get(DcMotor.class, Config.MOTORS[selected]);
        waitForStart();

        if (isStopRequested()) return;

        //Run the selected option
        while (opModeIsActive()) {
            speed = Math.min(1,Math.max(-1,speed + gamepad1.left_stick_y*.2)); //Speed can not go over 1 and go below -1
            motor.setPower(speed);

            telemetry.addLine("Use left stick to control motor speed");
            telemetry.addData("Motor", Config.MOTORS[selected]);
            telemetry.addData("Power", speed);
            telemetry.update();
        }

    }

}