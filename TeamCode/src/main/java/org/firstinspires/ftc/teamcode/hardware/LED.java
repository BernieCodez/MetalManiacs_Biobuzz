
package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver.LayerHeight;

import org.firstinspires.ftc.teamcode.prism.Color;
import org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.prism.PrismAnimations;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class LED {

    private final GoBildaPrismDriver prism;

    private Color previousLimelight1 = null;
    private Color previousLimelight2 = null;
    private String previousAllianceColor = "";

    public LED(HardwareMap hardwareMap) {

        prism = hardwareMap.get(
                GoBildaPrismDriver.class,
                "prism"
        );

        prism.setStripLength(36);
        prism.clearAllAnimations();
    }

    public void update(boolean tVisible, String allianceColor) {

        Color color = tVisible ? Color.GREEN : Color.RED;

        // STRIP 1: LIMELIGHT 1 (LEDs 0-11)

        if (color != previousLimelight1) {

            PrismAnimations.Solid limelight1 =
                    new PrismAnimations.Solid(color, 0, 11);

            limelight1.setBrightness(50);

            prism.insertAndUpdateAnimation(
                    LayerHeight.LAYER_0,
                    limelight1
            );

            previousLimelight1 = color;
        }

        // STRIP 2: ALLIANCE COLOR (LEDs 12-23)

        if (!allianceColor.equals(previousAllianceColor)) {

            if (allianceColor.equals("Blue")) {

                PrismAnimations.Solid allianceLEDs =
                        new PrismAnimations.Solid(Color.BLUE, 12, 23);

                allianceLEDs.setBrightness(50);

                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_1,
                        allianceLEDs
                );


            } else if (allianceColor.equals("Red")) {

                PrismAnimations.Solid allianceLEDs =
                        new PrismAnimations.Solid(Color.RED, 12, 23);

                allianceLEDs.setBrightness(50);

                prism.insertAndUpdateAnimation(
                        LayerHeight.LAYER_1,
                        allianceLEDs
                );
            }

            previousAllianceColor = allianceColor;
        }

        // STRIP 3: LIMELIGHT 2 (LEDs 24-35)

        if (color != previousLimelight2) {

            PrismAnimations.Solid limelight2 =
                    new PrismAnimations.Solid(color, 24, 35);

            limelight2.setBrightness(50);

            prism.insertAndUpdateAnimation(
                    LayerHeight.LAYER_2,
                    limelight2
            );

            previousLimelight2 = color;
        }
    }
    public void turnOff() {
        prism.clearAllAnimations();
    }
}

