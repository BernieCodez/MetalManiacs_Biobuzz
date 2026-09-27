package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

import java.util.Arrays;

@TeleOp(name = "ElementArrayCheck", group = "Test")
public class LED extends LinearOpMode {

    private GoBildaPrismDriver prism;
    private OuttakeController outtake;
    private RumbleGamepad driver;

    private String lastArray = "";

    @Override
    public void runOpMode() throws InterruptedException {

        prism = hardwareMap.get(
                GoBildaPrismDriver.class,
                "prism"
        );

        outtake = new OuttakeController(hardwareMap);

        driver = new RumbleGamepad(gamepad1);

        /*
         * ============================================================
         * TURN ALL 13 LEDs TRANSPARENT
         * ============================================================
         */

        prism.clearAllAnimations();

        PrismAnimations.Solid transparent =
                new PrismAnimations.Solid(Color.TRANSPARENT);

        transparent.setBrightness(100);
        transparent.setStartIndex(0);
        transparent.setStopIndex(12);

        prism.insertAndUpdateAnimation(
                LayerHeight.LAYER_0,
                transparent
        );

        telemetry.addLine("PRISM LED TEST");
        telemetry.addLine("--------------------");
        telemetry.addData(
                "LED Count",
                prism.getNumberOfLEDs()
        );
        telemetry.addLine("All LEDs set to TRANSPARENT");
        telemetry.addLine("Waiting for START...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        /*
         * Don't immediately clear the transparent animation.
         *
         * It stays active until the element array changes.
         */

        lastArray = Arrays.toString(outtake.elements);

        while (opModeIsActive()) {

            driver.update();

            /*
             * Update the outtake controller.
             */
            outtake.update(null, null);

            /*
             * A = fire / remove oldest element.
             */
            if (driver.wasJustPressed(
                    RumbleGamepad.Button.A)) {

                outtake.fire();
            }

            /*
             * Check if the element array changed.
             */
            String currentArray =
                    Arrays.toString(outtake.elements);

            if (!currentArray.equals(lastArray)) {

                updateLEDs(outtake.elements);

                lastArray = currentArray;
            }

            telemetry.addData(
                    "Array",
                    Arrays.toString(outtake.elements)
            );

            telemetry.update();

            sleep(20);
        }
    }

    /*
     * ================================================================
     * UPDATE LEDS
     * ================================================================
     *
     * Newest element = LED 0
     *
     * Example:
     *
     * Array:
     * [Blue Nectar, Red Nectar, Pollen]
     *
     * LED 0 = Pollen
     * LED 1 = Red Nectar
     * LED 2 = Blue Nectar
     */
    private void updateLEDs(String[] elements)
            throws InterruptedException {

        prism.clearAllAnimations();

        for (int ledIndex = 0;
             ledIndex < 10;
             ledIndex++) {

            Color color = Color.TRANSPARENT;

            if (ledIndex < elements.length) {

                int arrayIndex =
                        elements.length - 1 - ledIndex;

                color = getElementColor(
                        elements[arrayIndex]
                );
            }

            PrismAnimations.Solid led =
                    new PrismAnimations.Solid(color);

            led.setBrightness(50);
            led.setStartIndex(ledIndex);
            led.setStopIndex(ledIndex);

            insertLED(ledIndex, led);

            sleep(10);
        }
    }

    /*
     * ================================================================
     * ELEMENT -> COLOR
     * ================================================================
     */
    private Color getElementColor(String element) {

        if (element.equals("Red Nectar")) {
            return Color.RED;
        }

        if (element.equals("Blue Nectar")) {
            return Color.BLUE;
        }

        if (element.equals("Pollen")) {
            return Color.YELLOW;
        }

        return Color.TRANSPARENT;
    }

    /*
     * ================================================================
     * INSERT INTO PRISM LAYER
     * ================================================================
     */
    private void insertLED(
            int index,
            PrismAnimations.Solid led) {

        switch (index) {

            case 0:
                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_0,
                        led
                );
                break;

            case 1:
                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_1,
                        led
                );
                break;

            case 2:
                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_2,
                        led
                );
                break;

            case 3:
                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_3,
                        led
                );
                break;

            case 4:
                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_4,
                        led
                );
                break;

            case 5:
                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_5,
                        led
                );
                break;

            case 6:
                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_6,
                        led
                );
                break;

            case 7:
                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_7,
                        led
                );
                break;

            case 8:
                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_8,
                        led
                );
                break;

            case 9:
                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_9,
                        led
                );
                break;
        }
    }
}