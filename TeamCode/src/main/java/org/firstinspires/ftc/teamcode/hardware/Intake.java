package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config;

public class Intake {
    private DcMotorEx intake;
    double MAX_POWER = 0.7;

    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotorEx.class, Config.INTAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setPower(double power){
        intake.setPower(power);
    }

    public void stop(){
        intake.setPower(0);
    }

    public double getPower(){
        return intake.getPower();
    }
}
