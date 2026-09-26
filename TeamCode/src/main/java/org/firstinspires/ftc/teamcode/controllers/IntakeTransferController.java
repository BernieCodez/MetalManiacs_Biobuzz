package org.firstinspires.ftc.teamcode.controllers;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.hardware.Gate;
import org.firstinspires.ftc.teamcode.hardware.IntakeTransfer;
public class IntakeTransferController {
    IntakeTransfer intakeTransfer;
    Gate gate;
    public IntakeTransferController(HardwareMap hardwareMap){
        intakeTransfer = new IntakeTransfer(hardwareMap);
        gate = new Gate(hardwareMap);

    }
    public void runIntake(){
        intakeTransfer.setPower(0.5);
        }
    public void runGateOpen(){
        gate.open();
        }
    public void runGateClose() { gate.close(); }



    }
