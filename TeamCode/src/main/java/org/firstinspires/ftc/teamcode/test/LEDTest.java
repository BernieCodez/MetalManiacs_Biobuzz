//Only for teleop testing

package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.Element;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ElementArrayCheck", group = "Test")
public class LEDTest extends LinearOpMode {

    private GoBildaPrismDriver prism;
    private OuttakeController outtake;
    private RumbleGamepad driver;

    private final List<Element> lastElements = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {

        prism = hardwareMap.get(
                GoBildaPrismDriver.class,
                "prism"
        );

        outtake = new OuttakeController(hardwareMap);

        driver = new RumbleGamepad(gamepad1);

        /*
         * Reset all 12 LEDs to transparent before starting.
         */
        prism.clearAllAnimations();

        updateLEDs(new ArrayList<>());

        sleep(300);

        prism.clearAllAnimations();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        /*
         * Force the first LED update.
         */
        updateLEDs(getElements());

        lastElements.clear();
        lastElements.addAll(getElements());

        while (opModeIsActive()) {

            driver.update();

            outtake.update(null, null);

            if (driver.wasJustPressed(
                    RumbleGamepad.Button.A)) {

                outtake.fire();
            }

            List<Element> currentElements = getElements();

            /*
             * Only update the Prism when the element array
             * actually changes.
             */
            if (!currentElements.equals(lastElements)) {

                updateLEDs(currentElements);

                lastElements.clear();
                lastElements.addAll(currentElements);
            }

            telemetry.addData(
                    "Elements",
                    currentElements
            );

            telemetry.update();

            sleep(50);
        }
    }

    /**
     * Gets the current element list from the OuttakeController.
     */
    private List<Element> getElements() {

        List<Element> result = new ArrayList<>();

        /*
         * The controller's elementList is the source of truth.
         */
        result.addAll(outtake.elementList);

        return result;
    }

    /**
     * Updates all 12 LEDs in batches.
     *
     * 4 balls maximum:
     *
     * Ball 0 = LEDs 0-2
     * Ball 1 = LEDs 3-5
     * Ball 2 = LEDs 6-8
     * Ball 3 = LEDs 9-11
     *
     * Newest ball is displayed first.
     */
    private void updateLEDs(List<Element> elements)
            throws InterruptedException {

        /*
         * Clear old animations first.
         */
        prism.clearAllAnimations();

        /*
         * Maximum of 4 balls.
         */
        int ballCount = Math.min(elements.size(), 4);

        /*
         * Newest element goes first.
         */
        for (int ball = 0; ball < ballCount; ball++) {

            int elementIndex =
                    elements.size() - 1 - ball;

            Element element =
                    elements.get(elementIndex);

            Color color =
                    getElementColor(element);

            /*
             * Each ball gets exactly 3 LEDs.
             */
            int startLED = ball * 3;
            int stopLED = startLED + 2;

            PrismAnimations.Solid led =
                    new PrismAnimations.Solid();

            led.setPrimaryColor(color);
            led.setBrightness(50);

            led.setStartIndex(startLED);
            led.setStopIndex(stopLED);

            /*
             * Each 3-LED block gets its own layer.
             */
            insertLEDGroup(ball, led);

            /*
             * Small delay keeps the I2C traffic
             * from overwhelming the Prism.
             */
            sleep(15);
        }
    }

    /**
     * Converts an Element enum into a Prism color.
     */
    private Color getElementColor(Element element) {

        if (element == Element.RED_NECTAR) {
            return Color.RED;
        }

        if (element == Element.BLUE_NECTAR) {
            return Color.BLUE;
        }

        if (element == Element.POLLEN) {
            return Color.YELLOW;
        }

        /*
         * NONE / unknown = transparent.
         */
        return Color.TRANSPARENT;
    }

    /**
     * Sends one 3-LED block to its Prism animation layer.
     */
    private void insertLEDGroup(
            int group,
            PrismAnimations.Solid led) {

        switch (group) {

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
        }
    }
}