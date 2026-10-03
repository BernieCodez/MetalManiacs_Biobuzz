package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

//v uncommenting this will hide it on the driver station
//@Disabled
@TeleOp(name = "Move Flywheel Servo Test", group = "A - Test")
public class MovingFlywheelServoTest extends LinearOpMode {
    CRServo leftServo;
    CRServo rightServo;
    DcMotor leftFlywheel;
    DcMotor rightFlywheel;
    RumbleGamepad driver;
    public double position;
    public double speed;

    @Override
    public void runOpMode() {

        boolean previousSelectionInput = false;

//        leftServo = hardwareMap.get(Servo.class, "leftServo");
//        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        rightServo.setDirection(CRServo.Direction.REVERSE);

//        rightServo.setDirection(Servo.Direction.REVERSE);

        rightFlywheel = hardwareMap.get(DcMotor.class, "rightFlywheel");
        leftFlywheel = hardwareMap.get(DcMotor.class, "leftFlywheel");
        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        driver = new RumbleGamepad(gamepad1);
        position = 0;
        speed = 0.75;
        double offset = 0;

        waitForStart();

        if (isStopRequested()) return;

        //Run the selected option
        while (opModeIsActive()) {
            driver.update();

            position -= driver.leftY() * 0.01;

            // Keep position between 0 and 1
            position = Math.max(.15, Math.min(.85, position));

//            rightServo.setPosition(position-0.0);
//            leftServo.setPosition(position-.01);
            rightServo.setPower(position);
            leftServo.setPower(position);

            if (driver.wasJustPressed(RumbleGamepad.Button.RIGHT_BUMPER)){
                leftServo.setPower(0.1);
            } else if (driver.wasJustPressed(RumbleGamepad.Button.LEFT_BUMPER)) {
                leftServo.setPower(-0.1);
            }




            if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_UP)){
                speed += 0.05;
            } else if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_DOWN)) {
                speed-=0.05;
            }
            rightFlywheel.setPower(speed);
            leftFlywheel.setPower(speed);

            telemetry.addLine("Use left stick to control servo position");
            telemetry.addData("Position", position);
//            telemetry.addData("right servo position", rightServo.getPosition());
//            telemetry.addData("left servo position", leftServo.getPosition());
            telemetry.addData("offset",offset);
            telemetry.addData("speed", speed);

            telemetry.update();
        }

    }

}