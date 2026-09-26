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
    public void runIntake(double power1){
        intakeTransfer.setPower(0.5);
        }
    public void runGateOpen(double power2){
        gate.setPower(0.25);
        }
    public void runGateClose(double power3) { gate.setPower(-0.25); }



    }
