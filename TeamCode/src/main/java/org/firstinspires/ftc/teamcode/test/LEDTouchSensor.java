package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

@TeleOp(name = "LED Touch Sensor", group = "Test")
public class LEDTouchSensor extends LinearOpMode {

    public GoBildaPrismDriver prism;
    public TouchSensor touchSensor;

    // ==========================================
    // PURPLE ANIMATION
    // ==========================================

    public final PrismAnimations.Solid purple =
            new PrismAnimations.Solid(Color.PURPLE);


    @Override
    public void runOpMode() {

        // ==========================================
        // HARDWARE
        // ==========================================

        prism = hardwareMap.get(
                GoBildaPrismDriver.class,
                "prism"
        );

        touchSensor = hardwareMap.get(
                TouchSensor.class,
                "touch"
        );


        // ==========================================
        // PURPLE SETTINGS
        // ==========================================

        purple.setBrightness(50);
        purple.setStartIndex(0);
        purple.setStopIndex(12);


        // ==========================================
        // INIT TELEMETRY
        // ==========================================

        telemetry.addLine("PRISM LED TOUCH SENSOR TEST");
        telemetry.addLine("----------------------------");

        telemetry.addData(
                "Device ID",
                prism.getDeviceID()
        );

        telemetry.addData(
                "Firmware",
                prism.getFirmwareVersionString()
        );

        telemetry.addData(
                "Hardware",
                prism.getHardwareVersionString()
        );

        telemetry.addData(
                "LED Count",
                prism.getNumberOfLEDs()
        );

        telemetry.addLine();
        telemetry.addLine("Press START...");
        telemetry.update();


        // ==========================================
        // WAIT FOR START
        // ==========================================

        waitForStart();

        if (isStopRequested()) {
            return;
        }


        // ==========================================
        // MAIN LOOP
        // ==========================================

        while (opModeIsActive()) {

            // ==========================================
            // TOUCH SENSOR PRESSED
            // ==========================================

            if (touchSensor.isPressed()) {

                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_0,
                        purple
                );

            }

            // ==========================================
            // TOUCH SENSOR RELEASED
            // ==========================================

            else {

                prism.clearAllAnimations();

            }


            // ==========================================
            // TELEMETRY
            // ==========================================

            telemetry.addLine("PRISM LED TOUCH SENSOR TEST");
            telemetry.addLine("----------------------------");

            telemetry.addData(
                    "Touch Sensor",
                    touchSensor.isPressed() ? "PRESSED" : "RELEASED"
            );

            telemetry.addData(
                    "LED Count",
                    prism.getNumberOfLEDs()
            );

            telemetry.addData(
                    "FPS",
                    prism.getCurrentFPS()
            );

            telemetry.update();

            sleep(20);
        }
    }
}