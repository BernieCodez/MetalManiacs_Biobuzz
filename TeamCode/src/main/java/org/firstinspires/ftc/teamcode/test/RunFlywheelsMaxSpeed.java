package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

//v uncommenting this will hide it on the driver station
//@Disabled
@TeleOp(name = "Run Flywheels Max Speed", group = "A - Test")
public class RunFlywheelsMaxSpeed extends LinearOpMode {
    DcMotor leftFlywheel;
    DcMotor rightFlywheel;
    RumbleGamepad driver;
    public double speed;

    @Override
    public void runOpMode() {

        boolean previousSelectionInput = false;

        rightFlywheel = hardwareMap.get(DcMotor.class, "rightFlywheel");
        leftFlywheel = hardwareMap.get(DcMotor.class, "leftFlywheel");
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        if (isStopRequested()) return;

        //Run the selected option
        while (opModeIsActive()) {

            speed = 0.5;
            if (driver.wasJustPressed(RumbleGamepad.Button.RIGHT_BUMPER)){
                speed += 0.05;
            } else if (driver.wasJustPressed(RumbleGamepad.Button.LEFT_BUMPER)) {
                speed-=0.05;
            }

            rightFlywheel.setPower(speed);
            leftFlywheel.setPower(speed);

            telemetry.addLine("Use left stick to control motor speed");
            telemetry.addData("Speed", speed);
            telemetry.update();
        }

    }

}