package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad.*;
public class Profile {
    public Trigger FIRE = Trigger.RIGHT_TRIGGER;
    public Button ALLIANCE_COLOR = Button.START;
    public Button FIELD_CENTRIC = Button.LEFT_STICK;
    public Button REMOVE_DRIFT = Button.OPTION;
    public Button INTAKE_ON = Button.DPAD_UP;
    public Button REVERSE_INTAKE = Button.DPAD_DOWN;
    public Button TOGGLE_AUTOAIM = Button.A;

    public Profile(Trigger FIRE, Button ALLIANCE_COLOR, Button FIELD_CENTRIC, Button REMOVE_DRIFT, Button INTAKE_ON, Button REVERSE_INTAKE, Button TOGGLE_AUTOAIM){
        this.FIRE = FIRE;
        this.ALLIANCE_COLOR = ALLIANCE_COLOR;
        this.FIELD_CENTRIC = FIELD_CENTRIC;
        this.REMOVE_DRIFT = REMOVE_DRIFT;
        this.REVERSE_INTAKE = REVERSE_INTAKE;
        this.TOGGLE_AUTOAIM = TOGGLE_AUTOAIM;
    }
    public Profile(){}
}
