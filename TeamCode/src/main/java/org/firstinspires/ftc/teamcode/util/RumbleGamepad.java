package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.EnumMap;
import java.util.Map;

public class RumbleGamepad {

    private final Gamepad gamepad;

    public enum Button {
        CROSS,
        CIRCLE,
        SQUARE,
        TRIANGLE,

        A,
        B,
        X,
        Y,

        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,

        START,
        OPTION,
        PS,
        TOUCHPAD,

        RIGHT_BUMPER,
        LEFT_BUMPER,
        LEFT_STICK,
        RIGHT_STICK
    }

    //analog inputs (range of 0-1)
    public enum Trigger {
        LEFT_TRIGGER,
        RIGHT_TRIGGER
    }

    private final Map<Button, Boolean> currentStateButton =
            new EnumMap<>(Button.class);

    private final Map<Button, Boolean> previousStateButton =
            new EnumMap<>(Button.class);

    private final Map<Trigger, Boolean> currentStateTrigger =
            new EnumMap<>(Trigger.class);
    private final Map<Trigger, Boolean> previousStateTrigger =
            new EnumMap<>(Trigger.class);

    public RumbleGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;

        //save current state and previous state for each button
        for (Button button : Button.values()) {
            boolean state = getButtonState(button);

            currentStateButton.put(button, state);
            previousStateButton.put(button, state);
        }
        for (Trigger trigger : Trigger.values()) {
            boolean state = getTriggerState(trigger);

            currentStateTrigger.put(trigger, state);
            previousStateTrigger.put(trigger, state);
        }
    }




    //must be called at the start of the loop
    public void update() {
        for (Button button : Button.values()) {
            previousStateButton.put(button, currentStateButton.get(button));
            currentStateButton.put(button, getButtonState(button));
        }
        for (Trigger trigger : Trigger.values()) {
            previousStateTrigger.put(trigger, currentStateTrigger.get(trigger));
            currentStateTrigger.put(trigger, getTriggerState(trigger));
        }
    }


    //button currently held down
    public boolean isDown(Button button) {
        return currentStateButton.get(button);
    }

    public boolean isDown(Trigger trigger){return currentStateTrigger.get(trigger);}

    //triggers once when the button goes down

    public boolean wasJustPressed(Button button) {
        return currentStateButton.get(button) && !previousStateButton.get(button);
    }
    public boolean wasJustPressed(Trigger trigger){
        return currentStateTrigger.get(trigger) && !previousStateTrigger.get(trigger);
    }


    //triggers once when the button gets released
    public boolean wasJustReleased(Button button) {
        return !currentStateButton.get(button) && previousStateButton.get(button);
    }
    public boolean wasJustReleased(Trigger trigger) {
        return !currentStateTrigger.get(trigger) && previousStateTrigger.get(trigger);
    }


    //triggers once when the button state changes
    public boolean stateJustChanged(Button button) {
        return currentStateButton.get(button) != previousStateButton.get(button);
    }

    //USE READ VALUE ONLY WHEN NEEDED FOR DEBUG!
    public double readValue(Trigger trigger) {
        switch (trigger) {
            case LEFT_TRIGGER:
                return gamepad.left_trigger;

            case RIGHT_TRIGGER:
                return gamepad.right_trigger;

            default:
                return 0;
        }
    }


    //overloading for buttons!
    public boolean readValue(Button button) {
        return currentStateButton.get(button);
    }

    private boolean getTriggerState(Trigger trigger){
        switch (trigger){

            case LEFT_TRIGGER:
                return gamepad.left_trigger > 0;

            case RIGHT_TRIGGER:
                return gamepad.right_trigger > 0;

                default:
                return false;

        }
    }
    private boolean getButtonState(Button button) {
        switch (button) {

            case A:
            case CROSS:
                return gamepad.a;

            case B:
            case CIRCLE:
                return gamepad.b;

            case X:
            case SQUARE:
                return gamepad.x;

            case Y:
            case TRIANGLE:
                return gamepad.y;

            case DPAD_UP:
                return gamepad.dpad_up;

            case DPAD_DOWN:
                return gamepad.dpad_down;

            case DPAD_LEFT:
                return gamepad.dpad_left;

            case DPAD_RIGHT:
                return gamepad.dpad_right;

            case START:
                return gamepad.start;

            case OPTION:
                return gamepad.options;

            case PS:
                return gamepad.ps;

            case TOUCHPAD:
                return gamepad.touchpad;

            case RIGHT_BUMPER:
                return gamepad.right_bumper;

            case LEFT_BUMPER:
                return gamepad.left_bumper;

            case RIGHT_STICK:
                return gamepad.right_stick_button;

            case LEFT_STICK:
                return gamepad.left_stick_button;

            default:
                return false;
        }
    }

    //analog sticks
    public double leftX() {
        return gamepad.left_stick_x;
    }

    //positive is forward negative is backward
    public double leftY() {
        return -gamepad.left_stick_y;
    }

    public double rightX() {
        return gamepad.right_stick_x;
    }

    //positive is forward negative is backward
    public double rightY() {
        return -gamepad.right_stick_y;
    }


    //triggers
    public double leftTrigger() {
        return gamepad.left_trigger;
    }

    public double rightTrigger() {
        return gamepad.right_trigger;
    }


    //buttons
    public boolean cross() {
        return gamepad.a;
    }

    public boolean circle() {
        return gamepad.b;
    }

    public boolean square() {
        return gamepad.x;
    }

    public boolean triangle() {
        return gamepad.y;
    }

    public boolean dpadUp() {
        return gamepad.dpad_up;
    }

    public boolean dpadDown() {
        return gamepad.dpad_down;
    }

    public boolean dpadLeft() {
        return gamepad.dpad_left;
    }

    public boolean dpadRight() {
        return gamepad.dpad_right;
    }

    public boolean start() {
        return gamepad.start;
    }

    public boolean option() {
        return gamepad.options;
    }

    public boolean ps() {
        return gamepad.ps;
    }

    public boolean touchpad() {
        return gamepad.touchpad;
    }

    public boolean rightBumper() {return gamepad.right_bumper;}

    public boolean leftBumper() {return gamepad.left_bumper;}

    public boolean leftStick() {return gamepad.left_stick_button;}

    public boolean rightStick() {return gamepad.right_stick_button;}

    //leds & rumble :)
    public void light(int r, int g, int b) { //use 0-255 for rgb
        gamepad.setLedColor(
                Math.min(1,Math.max(r/255.0,0)),//has to divide because it wants 0-1 inputs...
                Math.min(1,Math.max(g/255.0,0)),
                Math.min(1,Math.max(b/255.0,0)),
                Integer.MAX_VALUE); //light effect lasts "forever"
    }

    public void light(int r, int g, int b, int ms) { //you can optionally define how long you want it to be
        gamepad.setLedColor(Math.min(1,Math.max(r/255.0,0)), Math.min(1,Math.max(g/255.0,0)), Math.min(1,Math.max(b/255.0,0)), ms);//has to divide because it wants 0-1 inputs...
    }

    public void stopRumble(){
        gamepad.stopRumble();
    }

    public void rumbleOne(){
        gamepad.rumble(1.0, 1.0, 5000);
    }
}
