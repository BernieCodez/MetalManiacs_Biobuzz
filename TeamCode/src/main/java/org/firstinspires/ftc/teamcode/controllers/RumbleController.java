package org.firstinspires.ftc.teamcode.controllers;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

public class RumbleController{
    private final RumbleGamepad rumbleGamepad;

    public RumbleController(RumbleGamepad rumbleGamepad) {
        this.rumbleGamepad = rumbleGamepad;
    }
    public void runRumble(){
        rumbleGamepad.rumbleOne();
    }
    public void stopRumble(){
        rumbleGamepad.stopRumble();
    }
}
