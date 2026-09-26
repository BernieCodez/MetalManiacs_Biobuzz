package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Gate;
import org.firstinspires.ftc.teamcode.hardware.Transfer;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

public class TransferController {
    Transfer transfer;
    Gate gate;
    public TransferController(HardwareMap hardwareMap){
        transfer = new Transfer(hardwareMap);
        gate = new Gate(hardwareMap);

    }
    public void setPower1(double power1){
        transfer.setPower(100);
        }
    public void setPower2(double power2){
        gate.setPower(50);
        }
    public void setPower3(double power3) { gate.setPower(-50); }



    }
