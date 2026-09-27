package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;
import static org.firstinspires.ftc.teamcode.Config.*;


import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Arrays;

public class LED {

    private GoBildaPrismDriver prism;
    private OuttakeController outtake;

    private ArrayList < Element > lastArray = null;

    private static final int TOTAL_LEDS = 12;
    private static final int LEDS_PER_ELEMENT = 3;

    public LED(HardwareMap hardwareMap) {

        prism = hardwareMap.get(
                GoBildaPrismDriver.class,
                "prism"
        );

        outtake = new OuttakeController(hardwareMap);
        prism.clearAllAnimations();

        PrismAnimations.Solid reset =
                new PrismAnimations.Solid(Color.TRANSPARENT);

        reset.setBrightness(100);
        reset.setStartIndex(0);
        reset.setStopIndex(TOTAL_LEDS - 1);

        prism.insertAndUpdateAnimation(
                LayerHeight.LAYER_0,
                reset
        );
    }

    public void update(ArrayList <Element>  currentArray) throws InterruptedException {

        if (!currentArray.equals(lastArray)) {

            updateLEDs(currentArray);

            lastArray = currentArray;
        }
    }

    private void updateLEDs(ArrayList <Element> elements)
            throws InterruptedException {

        prism.clearAllAnimations();

        int elementsToDisplay =
                Math.min(elements.size(), 4);

        for (int slot = 0; slot < 4; slot++) {

            Color color = Color.TRANSPARENT;

            if (slot < elementsToDisplay) {

                int arrayIndex =
                        elements.size() - 1 - slot;

                color = getElementColor(
                        elements.get(arrayIndex)
                );
            }

            int startLED =
                    slot * LEDS_PER_ELEMENT;

            int stopLED =
                    startLED + LEDS_PER_ELEMENT - 1;

            PrismAnimations.Solid led =
                    new PrismAnimations.Solid();

            led.setPrimaryColor(color);
            led.setBrightness(50);
            led.setStartIndex(startLED);
            led.setStopIndex(stopLED);

            insertLED(slot, led);

            Thread.sleep(15);
        }
    }

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

        return Color.TRANSPARENT;
    }

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
        }
    }
}