package org.firstinspires.ftc.teamcode.controllers;
import android.health.connect.datatypes.units.Mass;

import org.firstinspires.ftc.teamcode.util.RumbleGamepad;
import org.firstinspires.ftc.teamcode.hardware.IntakeTransfer;


public class RumbleController{
    private final RumbleGamepad rumbleGamepad;
    private final IntakeTransfer intakeTransfer;
    private boolean intakeOn = false;

    public RumbleController(RumbleGamepad rumbleGamepad, IntakeTransfer intakeTransfer) {
        this.rumbleGamepad = rumbleGamepad;
        this.intakeTransfer = intakeTransfer;
    }
    public void update(boolean intakeOn){
        if(Math.abs(intakeTransfer.getPower())>0.05){
            if(!intakeOn){
                rumbleGamepad.rumbleOne();
                intakeOn = true;
            }else{
                if (intakeOn){
                    rumbleGamepad.stopRumble();
                    intakeOn = false;
                }
            }
        }
    }

    public void runRumble(){
        rumbleGamepad.rumbleOne();
        intakeOn = true;
    }
    public void stopRumble(){
        rumbleGamepad.stopRumble();
        intakeOn = false;
    }

}

