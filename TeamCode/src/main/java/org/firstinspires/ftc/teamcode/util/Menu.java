
package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.function.Consumer;

import java.util.ArrayList;
import java.util.List;

public class Menu {
    private static double JOYSTICK_THRESHOLD = 0.75;

    public static class Screen {
        private final String title;
        private final List<Item> items = new ArrayList<>();

        private Screen parent;
        private int selected = 0;
        private Consumer<Telemetry> displayData;


        public Screen(String title) {
            this.title = title;
        }

        public Screen addMenu(String name, Screen submenu) {
            submenu.parent = this;
            items.add(new Item(name, submenu, null));
            return this;
        }

        public Screen addAction(String name, Runnable action) {
            items.add(new Item(name, null, action));
            return this;
        }

        public Screen addDisplay(Consumer<Telemetry> displayData) {
            this.displayData = displayData;
            return this;
        }
    }

    private static class Item {
        final String name;
        final Screen submenu;
        final Runnable action;

        Item(String name, Screen submenu, Runnable action) {
            this.name = name;
            this.submenu = submenu;
            this.action = action;
        }
    }

    private Screen current;
    private boolean previousStickUp;
    private boolean previousStickDown;
    public Menu(Screen root) {
        current = root;
    }

    public void update(RumbleGamepad gp) {
        gp.update();
        //dpad for nav
        boolean up = gp.wasJustPressed(RumbleGamepad.Button.DPAD_UP);
        boolean down = gp.wasJustPressed(RumbleGamepad.Button.DPAD_DOWN);

        //also has left stick for nav
        double stickY = gp.leftY();

        boolean stickDown = stickY < -JOYSTICK_THRESHOLD;
        boolean stickUp = stickY > JOYSTICK_THRESHOLD;

        //check if stick is valid
        boolean joystickUp = stickUp && !previousStickUp;
        boolean joystickDown = stickDown && !previousStickDown;

        previousStickUp = stickUp;
        previousStickDown = stickDown;

        //updated selected
        if (up || joystickUp) {
            current.selected--;
        } else if (down || joystickDown) {
            current.selected++;
        }

        //keep selection in bounds
        if (!current.items.isEmpty()) {
            current.selected = Math.max(
                    0,
                    Math.min(current.items.size() - 1, current.selected)
            );
        } else {
            current.selected = 0;
        }

        //confirm selection
        if (gp.wasJustPressed(RumbleGamepad.Button.A) && !current.items.isEmpty()) {

            Item item = current.items.get(current.selected);

            if (item.submenu != null) {
                current = item.submenu;
            } else if (item.action != null) {
                item.action.run();
                current = current.parent;
            }
        }

        //go back to previous page
        if (gp.wasJustPressed(RumbleGamepad.Button.B) && current.parent != null) {
            current = current.parent;
        }
    }

    public void render(Telemetry telemetry) {
        telemetry.clear();
        telemetry.addLine("=== " + current.title + " ===");

        //display custom data
        if (current.displayData != null) {
            current.displayData.accept(telemetry);
            telemetry.addLine("");
        }

        for (int i = 0; i < current.items.size(); i++) {
            Item item = current.items.get(i);

            telemetry.addLine(
                    (i == current.selected ? "> " : "  ")
                            + item.name
            );
        }

        telemetry.addLine("");
        telemetry.addLine("D-pad / Left Stick: Navigate");
        telemetry.addLine("A: Select");
        telemetry.addLine("B: Back");
        telemetry.addLine("");
        telemetry.addLine("Press Play To Proceed");

        telemetry.update();
    }
}
