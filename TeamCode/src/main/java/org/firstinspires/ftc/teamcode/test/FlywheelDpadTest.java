package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

@TeleOp(name = "Flywheel Dpad Test", group = "A - Test")
public class FlywheelDpadTest extends LinearOpMode {
    DcMotor leftFlywheel;
    DcMotor rightFlywheel;
    RumbleGamepad driver;
    public double speed;

    @Override
    public void runOpMode() {

        rightFlywheel = hardwareMap.get(DcMotor.class, Config.RIGHT_FLYWHEEL);
        leftFlywheel = hardwareMap.get(DcMotor.class, Config.LEFT_FLYWHEEL);
        driver = new RumbleGamepad(gamepad1);
        speed = -0.5;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            driver.update();
            if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_UP)) {
                speed -= 0.05;
            } else if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_DOWN)) {
                speed += 0.05;
            } else if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_RIGHT)) {
                speed -= 0.01;
            } else if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_LEFT)) {
                speed += 0.01;
            }
            speed = Range.clip(speed, -1, 0);

            rightFlywheel.setPower(speed);
            leftFlywheel.setPower(speed);

            telemetry.addLine("Dpad up/down: +/- 0.05");
            telemetry.addLine("Dpad right/left: +/- 0.01");
            telemetry.addData("Speed", "%.2f", speed);
            telemetry.update();
        }

        rightFlywheel.setPower(0);
        leftFlywheel.setPower(0);
    }

}
