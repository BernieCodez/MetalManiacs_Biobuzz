package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Config;

@TeleOp(name = "Run Motor", group = "Test")
public class RunMotor extends LinearOpMode {

    private int selected = 0;

    private double speed = 1;
    private boolean previousUp;
    private boolean previousDown;
    DcMotor motor;

    @Override
    public void runOpMode() {

        // Hardware initialization
        // motor = hardwareMap.get(...);

        while (opModeInInit()) {

            if (gamepad1.dpad_down && !previousDown) {
                selected++;
                previousDown = true;
            }

            if (gamepad1.dpad_up && !previousUp) {
                selected--;
                previousUp = true;
            }

            if (gamepad1.left_stick_y < -0.25 && !previousUp) {
                selected--;
                previousUp = true;
            }

            if (gamepad1.left_stick_y > 0.25 && !previousDown) {
                selected++;
                previousDown = true;
            }

            if (0.1 >= gamepad1.left_stick_y && gamepad1.left_stick_y >= -0.1){
                previousUp = false;
                previousDown = false;
            }

            selected = Math.max(0,Math.min(Config.MOTORS.length - 1, selected));

            //Display menu
            telemetry.addLine("=== SELECT MOTOR ===");

            for (int i = 0; i < Config.MOTORS.length; i++) {
                if (i == selected) {
                    telemetry.addLine("> " + Config.MOTORS[i]);
                } else {
                    telemetry.addLine("  " + Config.MOTORS[i]);
                }
            }

            telemetry.addLine("");
            telemetry.addLine("D-Pad ^v or Left Joystick ^v: Select");
            telemetry.addLine("Press start to run! The currently selected motor will be run.");

            telemetry.update();

            //Pressing B / Circle exits the menu
            if (gamepad1.b) {
                break;
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
            telemetry.addData("Power", speed);
            telemetry.update();
        }

    }

}