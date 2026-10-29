package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad.*;
public class Profile {
    public Trigger FIRE = Trigger.RIGHT_TRIGGER;
    public Button ALLIANCE_COLOR = Button.START;
    public Button FIELD_CENTRIC = Button.LEFT_STICK;
    public Button REMOVE_DRIFT = Button.X;
    public Button INTAKE_ON = Button.DPAD_UP;
    public Button REVERSE_INTAKE = Button.DPAD_DOWN;
    public Button TOGGLE_AUTOAIM = Button.A;
    public Button INTAKE_FLOWER_TOGGLE = Button.B;

    public String name;

    public Profile(){}

    public Profile(String name){
        this.name = name;
    }
}
