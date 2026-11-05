package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Config.*;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeTransfer {
    public static DcMotorEx intakeTransfer;

    public IntakeTransfer(HardwareMap hardwareMap){
        intakeTransfer = hardwareMap.get(DcMotorEx.class, INTAKE_TRANSFER);
        intakeTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeTransfer.setCurrentAlert(5,CurrentUnit.AMPS);
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

    public boolean isJamming(){
        return intakeTransfer.isOverCurrent();
    }

    public double getCurrent(){
        return intakeTransfer.getCurrent(CurrentUnit.AMPS);
    }
}
