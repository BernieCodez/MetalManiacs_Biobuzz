package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "ColorSensor", group = "Sensors")
public class ColorSensor extends LinearOpMode {

    // Keeps track of the detected color
    String detectedColor = "Unknown";

    @SuppressLint("Range")
    @Override
    public void runOpMode() {

        RevColorSensorV3 colorSensor =
                hardwareMap.get(RevColorSensorV3.class, "color");

        waitForStart();

        while (opModeIsActive()) {

            // Get RGB values
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            int alpha = colorSensor.alpha();

            double distance = colorSensor.getDistance(DistanceUnit.CM);

            // Find the largest RGB value
            int max = Math.max(red, Math.max(green, blue));

            // Prevent division by zero
            if (max == 0) {
                max = 1;
            }

            // Normalize RGB values to 0-255
            int normalizedRed = (red * 255) / max;
            int normalizedGreen = (green * 255) / max;
            int normalizedBlue = (blue * 255) / max;

            // Create HSV array
            float[] hsvValues = new float[3];

            // Convert normalized RGB to HSV
            Color.RGBToHSV(
                    normalizedRed,
                    normalizedGreen,
                    normalizedBlue,
                    hsvValues
            );

            float hue = hsvValues[0];
            float saturation = hsvValues[1];
            float value = hsvValues[2];

            // Detect the color
            detectedColor = detectColor(hue, saturation, value);

            // =========================
            // TELEMETRY
            // =========================

            telemetry.addData("Raw Red", red);
            telemetry.addData("Raw Green", green);
            telemetry.addData("Raw Blue", blue);

            telemetry.addData("Normalized Red", normalizedRed);
            telemetry.addData("Normalized Green", normalizedGreen);
            telemetry.addData("Normalized Blue", normalizedBlue);

            telemetry.addData("Hue", hue);
            telemetry.addData("Saturation", saturation);
            telemetry.addData("Value", value);

            telemetry.addData("Alpha", alpha);
            telemetry.addData("Distance (cm)", distance);

            telemetry.addData("Detected Color", detectedColor);

            telemetry.update();
        }
    }

    // ==========================================
    // COLOR DETECTION FUNCTION
    // ==========================================

    private String detectColor(
            float hue,
            float saturation,
            float value) {

        // Not enough color information
        if (saturation < 0.25) {
            return "Unknown";
        }

        // Very dark
        if (value < 0.10) {
            return "Black";
        }

        // RED
        if (hue >= 330 || hue < 20) {
            return "Red";
        }

        // BROWN
        if (hue >= 15 && hue < 45 && value < 0.65) {
            return "Brown";
        }

        // YELLOW
        if (hue >= 45 && hue < 75) {
            return "Yellow";
        }

        // GREEN
        if (hue >= 75 && hue < 170) {
            return "Green";
        }

        // BLUE
        if (hue >= 200 && hue < 260) {
            return "Blue";
        }

        // PURPLE
        if (hue >= 260 && hue < 330) {
            return "Purple";
        }

        return "Unknown";
    }
}