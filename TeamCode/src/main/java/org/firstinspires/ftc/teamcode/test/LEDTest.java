// Only for teleop testing

package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config.Element;
import org.firstinspires.ftc.teamcode.prism.Color;
import org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ElementArrayCheck", group = "Test")
public class LEDTest extends LinearOpMode {

    private GoBildaPrismDriver prism;
    private OuttakeController outtake;
    private RumbleGamepad driver;

    private final List<Element> lastElements =
            new ArrayList<>();

    private boolean secondLEDGreen = false;

    private static final int TOTAL_LEDS = 24;

    private static final int BALL_START = 0;
    private static final int BALL_END = 11;

    private static final int STATUS_START = 12;
    private static final int STATUS_END = 23;

    @Override
    public void runOpMode() throws InterruptedException {

        prism = hardwareMap.get(
                GoBildaPrismDriver.class,
                "prism"
        );

        prism.setStripLength(TOTAL_LEDS);

        outtake = new OuttakeController(hardwareMap);

        driver = new RumbleGamepad(gamepad1);

        prism.clearAllAnimations();

        lightOff();

        sleep(300);

        prism.clearAllAnimations();

        waitForStart();

        if (isStopRequested()) {
            lightOff();
            return;
        }

        List<Element> currentElements =
                getElements();

        updateLEDs(
                currentElements,
                secondLEDGreen
        );

        lastElements.clear();
        lastElements.addAll(currentElements);

        while (opModeIsActive()) {

            driver.update();

            outtake.update(
                    null,
                    null
            );

            if (driver.wasJustPressed(
                    RumbleGamepad.Button.A)) {

                outtake.fire();
            }

            if (driver.wasJustPressed(
                    RumbleGamepad.Button.SQUARE)) {

                secondLEDGreen =
                        !secondLEDGreen;

                updateStatusLED(
                        secondLEDGreen
                );
            }

            currentElements =
                    getElements();

            if (!currentElements.equals(
                    lastElements)) {

                updateBallLEDs(
                        currentElements
                );

                lastElements.clear();
                lastElements.addAll(
                        currentElements
                );
            }

            telemetry.addData(
                    "Elements",
                    currentElements
            );

            telemetry.addData(
                    "Number of Balls",
                    currentElements.size()
            );

            telemetry.addData(
                    "Current Detection",
                    outtake.getRealTimeDetection()
            );

            telemetry.addData(
                    "LED 13-24",
                    secondLEDGreen
                            ? "GREEN"
                            : "RED"
            );

            telemetry.update();

            sleep(50);
        }

        lightOff();
    }

    private List<Element> getElements() {

        List<Element> result =
                new ArrayList<>();

        result.addAll(
                outtake.elementList
        );

        return result;
    }

    private void updateLEDs(
            List<Element> elements,
            boolean green
    ) throws InterruptedException {

        updateBallLEDs(elements);

        updateStatusLED(green);
    }

    private void updateBallLEDs(
            List<Element> elements
    ) throws InterruptedException {

        prism.clearAllAnimations();

        int ballCount =
                Math.min(elements.size(), 4);

        for (int ball = 0; ball < 4; ball++) {

            Color color =
                    Color.TRANSPARENT;

            if (ball < ballCount) {

                int elementIndex =
                        elements.size()
                                - 1
                                - ball;

                Element element =
                        elements.get(
                                elementIndex
                        );

                color =
                        getElementColor(
                                element
                        );
            }

            int startLED =
                    BALL_START
                            + (ball * 3);

            int stopLED =
                    startLED + 2;

            PrismAnimations.Solid led =
                    new PrismAnimations.Solid(
                            color,
                            startLED,
                            stopLED
                    );

            led.setBrightness(
                    color == Color.TRANSPARENT
                            ? 0
                            : 50
            );

            insertLEDGroup(
                    ball,
                    led
            );

            sleep(15);
        }

        updateStatusLED(secondLEDGreen);
    }

    private void updateStatusLED(
            boolean green
    ) {

        Color color =
                green
                        ? Color.GREEN
                        : Color.RED;

        PrismAnimations.Solid statusLED =
                new PrismAnimations.Solid(
                        color,
                        STATUS_START,
                        STATUS_END
                );

        statusLED.setBrightness(50);

        prism.insertAndUpdateAnimation(
                LayerHeight.LAYER_4,
                statusLED
        );
    }

    private Color getElementColor(
            Element element
    ) {

        if (element ==
                Element.RED_NECTAR) {

            return Color.RED;
        }

        if (element ==
                Element.BLUE_NECTAR) {

            return Color.BLUE;
        }

        if (element ==
                Element.POLLEN) {

            return Color.YELLOW;
        }

        return Color.TRANSPARENT;
    }

    private void insertLEDGroup(
            int group,
            PrismAnimations.Solid led
    ) {

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

    private void lightOff() {

        prism.clearAllAnimations();

        for (int layer = 0; layer <= 4; layer++) {

            PrismAnimations.Solid off =
                    new PrismAnimations.Solid(
                            Color.TRANSPARENT,
                            0,
                            TOTAL_LEDS - 1
                    );

            off.setBrightness(0);

            prism.insertAndUpdateAnimation(
                    getLayer(layer),
                    off
            );
        }
    }

    private GoBildaPrismDriver.LayerHeight getLayer(
            int index
    ) {

        switch (index) {

            case 0:
                return LayerHeight.LAYER_0;

            case 1:
                return LayerHeight.LAYER_1;

            case 2:
                return LayerHeight.LAYER_2;

            case 3:
                return LayerHeight.LAYER_3;

            case 4:
                return LayerHeight.LAYER_4;

            default:
                return LayerHeight.LAYER_0;
        }
    }
}