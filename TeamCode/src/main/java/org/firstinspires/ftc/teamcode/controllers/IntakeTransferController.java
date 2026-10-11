package org.firstinspires.ftc.teamcode.controllers;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.FlowerIntake;
import org.firstinspires.ftc.teamcode.hardware.Gate;
import org.firstinspires.ftc.teamcode.hardware.IntakeTransfer;
public class IntakeTransferController {
    IntakeTransfer intakeTransfer;
    FlowerIntake flowerIntake;
    Gate gate;
    public IntakeTransferController(HardwareMap hardwareMap){
        intakeTransfer = new IntakeTransfer(hardwareMap);
        flowerIntake = new FlowerIntake(hardwareMap);
        gate = new Gate(hardwareMap);

    }
    public void runIntake(){
        intakeTransfer.setPower(0.5);
        }
    public void runGateOpen(){
        gate.open();
        }
    public void runGateClose() { gate.close(); }
    public boolean isOn(){return Math.abs(intakeTransfer.getPower())>0.05;}

    public void toggleFlowerIntake(){
        flowerIntake.toggle();
    }
}
