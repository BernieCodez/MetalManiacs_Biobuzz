//this code was just for testing it btw if you want to learn this will help :)

package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.Direction;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

@TeleOp(name = "LED Example", group = "Test")
public class LEDExample extends LinearOpMode {

    public GoBildaPrismDriver prism;
    public RumbleGamepad driver;

    // ==========================================
    // ANIMATIONS
    // ==========================================

    // Purple
    public final PrismAnimations.Solid purple =
            new PrismAnimations.Solid(Color.PURPLE);

    // Orange
    public final PrismAnimations.Solid orange =
            new PrismAnimations.Solid(Color.ORANGE);

    // Rainbow
    public final PrismAnimations.Rainbow rainbow =
            new PrismAnimations.Rainbow(
                    0,
                    360,
                    0.5f,
                    Direction.Forward
            );

    // Purple sparkle
    public final PrismAnimations.Sparkle sparkle =
            new PrismAnimations.Sparkle(
                    Color.PURPLE,
                    Color.TRANSPARENT
            );

    // Police lights
    public final PrismAnimations.PoliceLights policeLights =
            new PrismAnimations.PoliceLights();

    // Droid scan
    public final PrismAnimations.DroidScan droidScan =
            new PrismAnimations.DroidScan(
                    Color.PURPLE,
                    Color.TRANSPARENT
            );

    // Pulse
    public final PrismAnimations.Pulse pulse =
            new PrismAnimations.Pulse(
                    Color.PURPLE,
                    Color.BLUE
            );

    @Override
    public void runOpMode() {

        // ==========================================
        // HARDWARE
        // ==========================================

        driver = new RumbleGamepad(gamepad1);

        prism = hardwareMap.get(
                GoBildaPrismDriver.class,
                "prism"
        );

        // ==========================================
        // PURPLE
        // ==========================================

        purple.setBrightness(50);
        purple.setStartIndex(0);
        purple.setStopIndex(12);

        // ==========================================
        // ORANGE
        // ==========================================

        orange.setBrightness(50);
        orange.setStartIndex(0);
        orange.setStopIndex(12);

        // ==========================================
        // RAINBOW
        // ==========================================

        rainbow.setBrightness(50);
        rainbow.setStartIndex(0);
        rainbow.setStopIndex(12);

        // ==========================================
        // SPARKLE
        // ==========================================

        sparkle.setBrightness(50);
        sparkle.setStartIndex(0);
        sparkle.setStopIndex(12);
        sparkle.setPeriod(100);
        sparkle.setSparkleProbability(16);

        // ==========================================
        // POLICE LIGHTS
        // ==========================================

        policeLights.setBrightness(50);
        policeLights.setStartIndex(0);
        policeLights.setStopIndex(12);
        policeLights.setPeriod(1000);

        // ==========================================
        // DROID SCAN
        // ==========================================

        droidScan.setBrightness(50);
        droidScan.setStartIndex(0);
        droidScan.setStopIndex(12);
        droidScan.setEyeWidth(3);
        droidScan.setTrailWidth(3);

        // ==========================================
        // PULSE
        // ==========================================

        pulse.setBrightness(50);
        pulse.setStartIndex(0);
        pulse.setStopIndex(12);
        pulse.setPeriod(1000);

        // ==========================================
        // INIT TELEMETRY
        // ==========================================

        telemetry.addLine("PRISM LED TEST");
        telemetry.addLine("--------------------");

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
        telemetry.addLine("Waiting for START...");
        telemetry.update();

        // ==========================================
        // WAIT FOR START
        // ==========================================

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // ==========================================
        // START WITH PURPLE
        // ==========================================

        prism.insertAndUpdateAnimation(
                LayerHeight.LAYER_0,
                purple
        );

        String currentAnimation = "PURPLE";

        // ==========================================
        // MAIN LOOP
        // ==========================================

        while (opModeIsActive()) {

            // --------------------------------------
            // CROSS = PURPLE
            // --------------------------------------

            if (driver.cross()) {

                if (!currentAnimation.equals("PURPLE")) {

                    prism.insertAndUpdateAnimation(
                            LayerHeight.LAYER_0,
                            purple
                    );

                    currentAnimation = "PURPLE";
                }
            }

            // --------------------------------------
            // CIRCLE = POLICE LIGHTS
            // --------------------------------------

            if (driver.circle()) {

                if (!currentAnimation.equals("POLICE LIGHTS")) {

                    prism.insertAndUpdateAnimation(
                            LayerHeight.LAYER_0,
                            policeLights
                    );

                    currentAnimation = "POLICE LIGHTS";
                }
            }

            // --------------------------------------
            // SQUARE = OFF
            // --------------------------------------

            if (driver.square()) {

                if (!currentAnimation.equals("OFF")) {

                    prism.clearAllAnimations();

                    currentAnimation = "OFF";
                }
            }

            // --------------------------------------
            // TRIANGLE = PURPLE SPARKLE
            // --------------------------------------

            if (driver.triangle()) {

                if (!currentAnimation.equals("PURPLE SPARKLE")) {

                    prism.insertAndUpdateAnimation(
                            LayerHeight.LAYER_0,
                            sparkle
                    );

                    currentAnimation = "PURPLE SPARKLE";
                }
            }

            // --------------------------------------
            // DPAD UP = DROID SCAN
            // --------------------------------------

            if (driver.dpadUp()) {

                if (!currentAnimation.equals("DROID SCAN")) {

                    prism.insertAndUpdateAnimation(
                            LayerHeight.LAYER_0,
                            droidScan
                    );

                    currentAnimation = "DROID SCAN";
                }
            }

            // --------------------------------------
            // DPAD DOWN = PULSE
            // --------------------------------------

            if (driver.dpadDown()) {

                if (!currentAnimation.equals("PULSE")) {

                    prism.insertAndUpdateAnimation(
                            LayerHeight.LAYER_0,
                            pulse
                    );

                    currentAnimation = "PULSE";
                }
            }

            // --------------------------------------
            // R2 = RAINBOW
            // --------------------------------------

            if (driver.rightTrigger() > 0.05) {

                if (!currentAnimation.equals("RAINBOW")) {

                    prism.insertAndUpdateAnimation(
                            LayerHeight.LAYER_0,
                            rainbow
                    );

                    currentAnimation = "RAINBOW";
                }
            }

            // --------------------------------------
            // L2 = ORANGE
            // --------------------------------------

            if (driver.leftTrigger() > 0.05) {

                if (!currentAnimation.equals("ORANGE")) {

                    prism.insertAndUpdateAnimation(
                            LayerHeight.LAYER_0,
                            orange
                    );

                    currentAnimation = "ORANGE";
                }
            }

            // ==========================================
            // TELEMETRY
            // ==========================================

            telemetry.addLine("PRISM LED TEST");
            telemetry.addLine("--------------------");

            telemetry.addData(
                    "Current Animation",
                    currentAnimation
            );

            telemetry.addData(
                    "LED Count",
                    prism.getNumberOfLEDs()
            );

            telemetry.addData(
                    "FPS",
                    prism.getCurrentFPS()
            );

            telemetry.addLine();

            // Controller debug
            telemetry.addData(
                    "Cross",
                    driver.cross()
            );

            telemetry.addData(
                    "Circle",
                    driver.circle()
            );

            telemetry.addData(
                    "Square",
                    driver.square()
            );

            telemetry.addData(
                    "Triangle",
                    driver.triangle()
            );

            telemetry.addData(
                    "DPad Up",
                    driver.dpadUp()
            );

            telemetry.addData(
                    "DPad Down",
                    driver.dpadDown()
            );

            telemetry.addData(
                    "L2",
                    "%.2f",
                    driver.leftTrigger()
            );

            telemetry.addData(
                    "R2",
                    "%.2f",
                    driver.rightTrigger()
            );

            telemetry.addLine();
            telemetry.addLine("CROSS = Purple");
            telemetry.addLine("CIRCLE = Police Lights");
            telemetry.addLine("SQUARE = OFF");
            telemetry.addLine("TRIANGLE = Purple Sparkle");
            telemetry.addLine("DPAD UP = Droid Scan");
            telemetry.addLine("DPAD DOWN = Pulse");
            telemetry.addLine("L2 = Orange");
            telemetry.addLine("R2 = Rainbow");

            telemetry.update();

            sleep(20);
        }
    }
}