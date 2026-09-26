package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.IntakeTransfer;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

public class RumbleController {


    public RumbleController(HardwareMap hardwareMap, RumbleGamepad gamepad){


        if (IntakeTransfer.getPower()>0){
            gamepad.rumbleOne();
        }

    }

}
