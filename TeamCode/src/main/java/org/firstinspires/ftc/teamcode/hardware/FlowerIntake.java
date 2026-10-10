
package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Flower Intake", group = "Test")
public class FlowerIntake extends LinearOpMode {

    private Servo flowerRamp;
    private boolean rampOpen = false;
    private boolean previousA = false;

    @Override
    public void runOpMode() {
        flowerRamp = hardwareMap.get(Servo.class, "flowerRamp");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean currentA = gamepad1.a;

            if (currentA && !previousA) {
                rampOpen = !rampOpen;

                flowerRamp.setPosition(rampOpen ? 1.0 : 0.0);
            }

            previousA = currentA;

            telemetry.addData("A Pressed", currentA);
            telemetry.addData("Ramp Open", rampOpen);
            telemetry.addData("Servo Position", flowerRamp.getPosition());
            telemetry.update();
        }
    }
}