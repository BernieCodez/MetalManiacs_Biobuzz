package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad.*;
public class Profile {
    public Trigger FIRE = Trigger.RIGHT_TRIGGER;
    public Button ALLIANCE_COLOR = Button.START;
    public Button RESET_YAW = Button.Y; //triangle
    public Button FIELD_CENTRIC = Button.LEFT_STICK;
    public Button REMOVE_DRIFT = Button.X; //square
    public Button INTAKE_FORWARD = Button.RIGHT_BUMPER;
    public Button INTAKE_REVERSE = Button.LEFT_BUMPER;
    public Button TOGGLE_AUTOAIM = Button.A; //cross
    public Button INTAKE_FLOWER_TOGGLE = Button.B; //circle

    public String name;

    public Profile(){}

    public Profile(String name){
        this.name = name;
    }
}
