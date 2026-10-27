
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

// @Disabled
@TeleOp(name = "Run Motor Pair", group = "A - Test")
public class RunMotorPair extends LinearOpMode {

    private int selected1 = 0;
    private int selected2 = 1;
    private int activeMenu = 0;

    private double speed = 0;

    private DcMotor motor1;
    private DcMotor motor2;
    public RumbleGamepad driver;

    @Override
    public void runOpMode() {

        boolean previousSelectionInput = false;
        driver = new RumbleGamepad(gamepad1);
        while (opModeInInit()) {
            boolean up = gamepad1.dpad_up
                    || gamepad1.left_stick_y < -0.25;

            boolean down = gamepad1.dpad_down
                    || gamepad1.left_stick_y > 0.25;

            // Switch between motor menus using left/right
            if (gamepad1.dpad_left) {
                activeMenu = 0;
            } else if (gamepad1.dpad_right) {
                activeMenu = 1;
            }

            // Navigate the currently active menu
            if ((up || down) && !previousSelectionInput) {
                int direction = up ? -1 : 1;

                if (activeMenu == 0) {
                    selected1 += direction;
                } else {
                    selected2 += direction;
                }
            }

            previousSelectionInput = up || down;

            // Keep both selections within the available range
            selected1 = Math.max(
                    0,
                    Math.min(Config.MOTORS.length - 1, selected1)
            );

            selected2 = Math.max(
                    0,
                    Math.min(Config.MOTORS.length - 1, selected2)
            );

            // Prevent selecting the same motor twice
            if (selected1 == selected2 && Config.MOTORS.length > 1) {
                if (activeMenu == 0) {
                    selected1 = Math.max(
                            0,
                            Math.min(Config.MOTORS.length - 1,
                                    selected1 + (up ? -1 : 1))
                    );
                } else {
                    selected2 = Math.max(
                            0,
                            Math.min(Config.MOTORS.length - 1,
                                    selected2 + (up ? -1 : 1))
                    );
                }
            }

            // Display both menus
            telemetry.addLine("=== RUN MOTOR PAIR ===");
            telemetry.addLine("");

            telemetry.addLine(
                    (activeMenu == 0 ? "> " : "  ")
                            + "MOTOR 1: " + Config.MOTORS[selected1]
            );

            telemetry.addLine(
                    (activeMenu == 1 ? "> " : "  ")
                            + "MOTOR 2: " + Config.MOTORS[selected2]
            );

            telemetry.addLine("");
            telemetry.addLine("D-Pad UP/DOWN: Select motor");
            telemetry.addLine("D-Pad LEFT/RIGHT: Switch menus");
            telemetry.addLine("Press START to run both motors");
            telemetry.addLine("Press B / Circle to cancel");

            telemetry.update();

            if (gamepad1.b) {
                return;
            }
        }

        // Initialize both selected motors
        motor1 = hardwareMap.get(
                DcMotor.class, Config.MOTORS[selected1]
        );

        motor2 = hardwareMap.get(
                DcMotor.class, Config.MOTORS[selected2]
        );

        waitForStart();

        if (isStopRequested()) return;

        boolean reverse = true;

        // Run both motors simultaneously
        while (opModeIsActive()) {
            driver.update();

            if(driver.wasJustPressed(RumbleGamepad.Button.A)){
                reverse = !reverse;
            }

            motor2.setDirection(reverse ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
            if(driver.wasJustPressed(RumbleGamepad.Button.RIGHT_BUMPER)){
                speed += 0.1;
            } else if (driver.wasJustPressed(RumbleGamepad.Button.LEFT_BUMPER)) {
                speed-=0.1;
            }
            speed = Math.max(-1, Math.min(1, speed));

            motor1.setPower(speed);
            motor2.setPower(speed);

            telemetry.addLine("Use dpad up/down to control motor speed");
            telemetry.addLine("Use A to reverse motor direction");
            telemetry.addData("Motor 1", Config.MOTORS[selected1]);
            telemetry.addData("Motor 2", Config.MOTORS[selected2]);
            telemetry.addData("Power", "%.2f", speed);
            telemetry.update();
        }

        // Stop both motors when the OpMode ends
        motor1.setPower(0);
        motor2.setPower(0);
    }
}
