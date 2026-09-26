package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

@TeleOp(name="LEDLightTest", group="Linear OpMode")

public class LED extends LinearOpMode {

    public RumbleGamepad driver;
    GoBildaPrismDriver prism;

    PrismAnimations.Solid purple =
            new PrismAnimations.Solid(Color.PURPLE);

    PrismAnimations.Solid orange =
            new PrismAnimations.Solid(Color.ORANGE);

    @Override
    public void runOpMode() {

        driver = new RumbleGamepad(gamepad1);

        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        // Purple settings
        purple.setBrightness(50);
        purple.setStartIndex(0);
        purple.setStopIndex(12);

        // Orange settings
        orange.setBrightness(50);
        orange.setStartIndex(0);
        orange.setStopIndex(12);

        telemetry.addLine("Prism Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Purple
            prism.insertAndUpdateAnimation(
                    LayerHeight.LAYER_0,
                    purple
            );

            sleep(500);

            // Orange
            prism.insertAndUpdateAnimation(
                    LayerHeight.LAYER_0,
                    orange
            );

            sleep(500);
        }
    }
}