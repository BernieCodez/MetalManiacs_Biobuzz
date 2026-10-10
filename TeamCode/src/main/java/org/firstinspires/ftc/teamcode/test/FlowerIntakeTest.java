
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.IntakeTransfer;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

@TeleOp(name = "Flower INTAKE TEST", group = "Test")
public class FlowerIntakeTest extends LinearOpMode {

    private Servo rightWedge;
    private Servo leftWedge;
    private IntakeTransfer intakeTransfer;

    public static final double MAX_POSITION = 1;
    public static final double MIN_POSITION = 0;
    public double position = 0;

    public RumbleGamepad driver;
    private boolean rampOpen = false;

    @Override
    public void runOpMode() {
        rightWedge = hardwareMap.get(Servo.class, "rightServo");
        leftWedge = hardwareMap.get(Servo.class, "leftServo");
        intakeTransfer = new IntakeTransfer(hardwareMap);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driver.update();

            if (driver.wasJustPressed(RumbleGamepad.Button.A)) {
                rampOpen = !rampOpen;

                position = rampOpen ? MAX_POSITION : MIN_POSITION;
            }

            if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_UP)) {
                position += 0.1;
            }else if(driver.wasJustPressed(RumbleGamepad.Button.DPAD_DOWN)){
                position -= 0.1;
            }

            if(driver.wasJustPressed(RumbleGamepad.Button.RIGHT_BUMPER)) {
                intakeTransfer.setPower(1);
            }
            if (driver.wasJustPressed(RumbleGamepad.Button.LEFT_BUMPER)) {
                intakeTransfer.setPower(-1);
            }
            if (driver.wasJustPressed(RumbleGamepad.Button.B)) {
                intakeTransfer.setPower(0);
            }
            position = Math.min(MAX_POSITION, Math.max(MIN_POSITION, position));

            leftWedge.setPosition(position);
            rightWedge.setPosition(position);

            telemetry.addData("Ramp Open", rampOpen);
            telemetry.addData("Servo Position", leftWedge.getPosition());
            telemetry.addData("Servo Position", rightWedge.getPosition());
            telemetry.update();
        }
    }
}