package org.firstinspires.ftc.teamcode.controllers;
import android.health.connect.datatypes.units.Mass;

import org.firstinspires.ftc.teamcode.util.RumbleGamepad;
import org.firstinspires.ftc.teamcode.hardware.IntakeTransfer;


public class RumbleController{
    private final RumbleGamepad rumbleGamepad;
    private final IntakeTransfer intakeTransfer;
    private boolean isRumbling = false;

    public RumbleController(RumbleGamepad rumbleGamepad, IntakeTransfer intakeTransfer) {
        this.rumbleGamepad = rumbleGamepad;
        this.intakeTransfer = intakeTransfer;
    }
    public void update(){
        if(Math.abs(intakeTransfer.getPower())>0.05){
            if(!isRumbling){
                rumbleGamepad.rumbleOne();
                isRumbling = true;
            }else{
                if (isRumbling){
                    rumbleGamepad.stopRumble();
                    isRumbling = false;
                }
            }
        }
    }

    public void runRumble(){
        rumbleGamepad.rumbleOne();
        isRumbling = true;
    }
    public void stopRumble(){
        rumbleGamepad.stopRumble();
        isRumbling = false;
    }

}

