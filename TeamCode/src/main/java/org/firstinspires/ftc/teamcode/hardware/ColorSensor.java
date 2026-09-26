package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.Config.*;

public class ColorSensor {

    public RevColorSensorV3 colorSensor;

    private final double redMin = 0.0;
    private final double redMax = 45.0;
    private final double yellowMin = 46.0;
    private final double yellowMax = 130.0;
    private final double blueMin = 140.0;
    private final double blueMax = 240.0;

    private final float[] hsvValues = {0F, 0F, 0F};
    public ColorSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, COLOR_SENSOR);
    }

    public Element getDetectedColor() {
        double distance = colorSensor.getDistance(DistanceUnit.CM);
        if (distance > 5.0) {
            return Element.NONE;
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
            return Element.RED_NECTAR;
        } else if (currentHue >= blueMin && currentHue <= blueMax) {
            return Element.BLUE_NECTAR;
        } else if (currentHue >= yellowMin && currentHue <= yellowMax) {
            return Element.POLLEN;
        }

        return Element.NONE;
    }
}
