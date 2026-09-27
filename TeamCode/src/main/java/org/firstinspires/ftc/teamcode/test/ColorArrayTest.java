package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.Config.Element;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

import java.util.Arrays;

@TeleOp(name = "ColorArrayTest", group = "Test")
public class ColorArrayTest extends LinearOpMode {

    private OuttakeController outtakeController;
    private RumbleGamepad driver;

    @Override
    public void runOpMode() throws InterruptedException {
        outtakeController = new OuttakeController(hardwareMap);

        driver = new RumbleGamepad(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            driver.update();

            outtakeController.update(null, null);

            if (driver.wasJustPressed(RumbleGamepad.Button.A)) {
                outtakeController.fire();
            }

            Element sensorRead = outtakeController.getRealTimeDetection();

            telemetry.addData("Sensor Seeing: ", sensorRead.toString());
            telemetry.addData("Elements: ", Arrays.toString(outtakeController.elements));
            telemetry.update();
        }
    }
}