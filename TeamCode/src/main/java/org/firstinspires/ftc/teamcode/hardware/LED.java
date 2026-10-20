package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver.LayerHeight;

import org.firstinspires.ftc.teamcode.prism.Color;
import org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.prism.PrismAnimations;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class LED {

    private final GoBildaPrismDriver prism;
    private Color previous = Color.RED;

    public LED(HardwareMap hardwareMap) {
        prism = hardwareMap.get(
                GoBildaPrismDriver.class,
                "prism"
        );

        prism.setStripLength(12);
        prism.clearAllAnimations();
    }

    public void update(boolean tVisible) {

        Color color = tVisible ? Color.GREEN : Color.RED;

        if (color != previous){
            PrismAnimations.Solid led = new PrismAnimations.Solid(color, 0, 11);
            led.setBrightness(50);

            prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, led);
        }
    }
}