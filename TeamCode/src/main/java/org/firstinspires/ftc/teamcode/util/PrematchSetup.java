
package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.Config.Alliance;
import org.firstinspires.ftc.teamcode.Profile;

@TeleOp(name = "Prematch Setup", group = "C - Util")
public class PrematchSetup extends LinearOpMode {
    private RumbleGamepad gp;

    @Override
    public void runOpMode() {
        gp = new RumbleGamepad(gamepad1);

        Menu.Screen root = new Menu.Screen("Prematch Setup")
                .addDisplay(telemetry -> {
                    telemetry.addData(
                            "Active Driver",
                            Drivers.activeDriver == null
                                    ? "Not selected"
                                    : Drivers.activeDriver.name
                    );

                    telemetry.addData(
                            "Alliance",
                            Config.currentAlliance == null
                                    ? "Not selected"
                                    : Config.currentAlliance.name()
                    );
                });

        Menu.Screen drivers = new Menu.Screen("Select Driver");
        Menu.Screen alliances = new Menu.Screen("Select Alliance");

        for (Profile driver : Drivers.DRIVERS) {
            drivers.addAction(driver.name, () -> {
                Drivers.activeDriver = driver;
            });
        }

        for (Alliance alliance : Alliance.values()) {
            alliances.addAction(alliance.name(), () -> {
                Config.currentAlliance = alliance;
            });
        }

        root.addMenu("Driver", drivers);
        root.addMenu("Alliance", alliances);

        Menu menu = new Menu(root);

        while (opModeInInit()) {
            menu.update(gp);
            menu.render(telemetry);

            if (gp.wasJustPressed(RumbleGamepad.Button.START)) {
                break;
            }

            idle();
        }
    }
}
