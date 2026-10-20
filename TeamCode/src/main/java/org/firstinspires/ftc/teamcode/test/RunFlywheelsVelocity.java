package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

//v uncommenting this will hide it on the driver station
@Disabled
@TeleOp(name = "Run Flywheels With Velocity", group = "A - Test")
public class RunFlywheelsVelocity extends LinearOpMode {
    DcMotorEx leftFlywheel;
    DcMotorEx rightFlywheel;
    RumbleGamepad driver;
    public double tps;
    public static double MIN_VELOCITY = 700;
    public static double MAX_VELOCITY = 2500;
    public static double VELOCITY_STEP = 100;

    @Override
    public void runOpMode() {
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "rightFlywheel");
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "leftFlywheel");
        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        driver = new RumbleGamepad(gamepad1);
        tps = 1000;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            driver.update();
            if (driver.wasJustPressed(RumbleGamepad.Button.RIGHT_BUMPER)){
                tps += VELOCITY_STEP;
            } else if (driver.wasJustPressed(RumbleGamepad.Button.LEFT_BUMPER)) {
                tps-=VELOCITY_STEP;
            }

            tps = Math.max(MIN_VELOCITY, Math.min(tps, MAX_VELOCITY));

            rightFlywheel.setVelocity(tps);
            leftFlywheel.setVelocity(tps);

            telemetry.addLine("Use bumpers  to control motor velocity");
            telemetry.addData("Target Velocity", tps);
            telemetry.addData("Actual Velocity", rightFlywheel.getVelocity());
            telemetry.addData("Velocity Error", tps - rightFlywheel.getVelocity());
            telemetry.update();
        }
    }
}