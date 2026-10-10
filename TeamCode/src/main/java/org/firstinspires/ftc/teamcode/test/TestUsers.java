package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.Profile;
import org.firstinspires.ftc.teamcode.util.Drivers;

//v uncommenting this will hide it on the driver station
//@Disabled
@TeleOp(name = "Test Users", group = "Z - Test")
public class TestUsers extends LinearOpMode {


    private int selected = 0;
    private boolean previousUp;
    private boolean previousDown;


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
                    Math.min(Config.MOTORS.length - 1, selected)
            );

            // Display menu
            telemetry.addLine("=== SELECT USER ===");

            for (int i = 0; i < Drivers.DRIVERS.length; i++) {
                if (i == selected) {
                    telemetry.addLine("> " + Drivers.DRIVERS[i]);
                } else {
                    telemetry.addLine("  " + Drivers.DRIVERS[i]);
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

        waitForStart();

        if (isStopRequested()) return;

        Drivers.activeDriver = Drivers.DRIVERS[selected];

        //Run the selected option
        while (opModeIsActive()) {


            telemetry.addLine("Use left stick to switch between users");
            telemetry.update();
        }



    }


}