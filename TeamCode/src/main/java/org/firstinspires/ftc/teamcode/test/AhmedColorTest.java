package org.firstinspires.ftc.teamcode.test;

import android.graphics.Color;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AhmedColorTest", group = "Autonomous")
public class AhmedColorTest extends LinearOpMode {

    private RevColorSensorV3 colorSensor;

    private final double redMin = 0.0;
    private final double redMax = 45.0;
    private final double yellowMin = 46.0;
    private final double yellowMax = 130.0;
    private final double blueMin = 140.0;
    private final double blueMax = 240.0;

    private final float[] hsvValues = {0F, 0F, 0F};

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color");

        waitForStart();

        while (opModeIsActive()) {
            String detectedColor = getDetectedColor();

            telemetry.clearAll();
            telemetry.addData("Color: ", detectedColor);
            telemetry.addData("Hue: ", hsvValues[0]);
            telemetry.addData("Saturation: ", hsvValues[1]);
            telemetry.update();

            idle();
        }
    }

    private String getDetectedColor() {
        double distance = colorSensor.getDistance(DistanceUnit.CM);
        if (distance > 5.0) {
            return "NOTHING (Too Far)";
        }

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsvValues
        );

        float currentHue = hsvValues[0];

        if (currentHue >= redMin && currentHue <= redMax) {
            return "RED";
        } else if (currentHue >= blueMin && currentHue <= blueMax) {
            return "BLUE";
        } else if (currentHue >= yellowMin && currentHue <= yellowMax) {
            return "YELLOW";
        }

        return "UNKNOWN COLOR";
    }
}
