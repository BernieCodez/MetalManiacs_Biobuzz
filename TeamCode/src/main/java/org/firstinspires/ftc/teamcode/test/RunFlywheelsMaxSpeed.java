package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Config;

//v uncommenting this will hide it on the driver station
//@Disabled
@TeleOp(name = "Run Flywheels Max Speed", group = "A - Test")
public class RunFlywheelsMaxSpeed extends LinearOpMode {
    DcMotor leftFlywheel;
    DcMotor rightFlywheel;
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
            speed = Math.min(1,Math.max(-1,speed + gamepad1.left_stick_y*.2)); //Speed can not go over 1 and go below -1
            rightFlywheel.setPower(speed);
            leftFlywheel.setPower(speed);

            telemetry.addLine("Use left stick to control motor speed");
            telemetry.addData("Speed", speed);
            telemetry.update();
        }

    }

}