package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config;

public class IntakeTransfer {
    private DcMotorEx intakeTransfer;
    double MAX_POWER = 0.7;

    public IntakeTransfer(HardwareMap hardwareMap){
        intakeTransfer = hardwareMap.get(DcMotorEx.class, Config.INTAKE_TRANSFER);
        intakeTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPower(double power){
        intakeTransfer.setPower(power);
    }

    public void stop(){
        intakeTransfer.setPower(0);
    }

    public double getPower(){
        return intakeTransfer.getPower();
    }
}
