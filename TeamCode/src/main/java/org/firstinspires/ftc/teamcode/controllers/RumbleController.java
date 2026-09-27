package org.firstinspires.ftc.teamcode.controllers;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;


public class RumbleController{
    private final RumbleGamepad rumbleGamepad;
    public RumbleController(RumbleGamepad rumbleGamepad) {
        this.rumbleGamepad = rumbleGamepad;
    }
    public void update(boolean intakeOn){
        if (intakeOn){
            rumbleGamepad.rumbleOne();
        }else {
            rumbleGamepad.stopRumble();
        }
    }
}

