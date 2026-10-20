package org.firstinspires.ftc.teamcode.controllers;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.FlowerIntake;
import org.firstinspires.ftc.teamcode.hardware.Gate;
import org.firstinspires.ftc.teamcode.hardware.IntakeTransfer;
public class IntakeTransferController {
    IntakeTransfer intakeTransfer;
    FlowerIntake flowerIntake;
    public IntakeTransferController(HardwareMap hardwareMap){
        intakeTransfer = new IntakeTransfer(hardwareMap);
        flowerIntake = new FlowerIntake(hardwareMap);
    }
    public void setPower(double power){
        intakeTransfer.setPower(power);
        }
    public boolean isOn(){return Math.abs(intakeTransfer.getPower())>0.05;}

    public void toggleFlowerIntake(){
        flowerIntake.toggle();
    }
}
