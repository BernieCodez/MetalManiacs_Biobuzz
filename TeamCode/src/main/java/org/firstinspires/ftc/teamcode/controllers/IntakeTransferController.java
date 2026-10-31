package org.firstinspires.ftc.teamcode.controllers;
import static org.firstinspires.ftc.teamcode.Config.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.FlowerIntake;
import org.firstinspires.ftc.teamcode.hardware.IntakeTransfer;
public class IntakeTransferController {
    IntakeTransfer intakeTransfer;
    FlowerIntake flowerIntake;
    public double power = 0;

    public IntakeTransferController(HardwareMap hardwareMap){
        intakeTransfer = new IntakeTransfer(hardwareMap);
        flowerIntake = new FlowerIntake(hardwareMap);
    }

    public void update(){
        intakeTransfer.setPower(power);
    }

    public void toggleForward(){
        if (isOn()){
            power = 0;
        } else {
            power = MAX_INTAKE_TRANSFER_POWER;
        }
    }

    public void toggleReverse(){
        if (isOn()){
            power = 0;
        } else {
            power = -MAX_INTAKE_TRANSFER_POWER;
        }
    }
    public void setPower(double power){
        this.power = power;
        }
    public boolean isOn(){return Math.abs(intakeTransfer.getPower())>0.05;}

    public void toggleFlowerIntake(){
        flowerIntake.toggle();
    }

    public void flowerIntakeDown(){
        flowerIntake.down();
    }

    public void flowerIntakeUp(){
        flowerIntake.up();
    }
}
