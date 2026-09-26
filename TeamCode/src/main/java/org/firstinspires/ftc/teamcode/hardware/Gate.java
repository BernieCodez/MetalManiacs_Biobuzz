package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config;

public class Gate {
    private Servo gate;
    double MAX_POWER = 0;

    public Gate(HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, Config.GATE);
        gate.setDirection(Servo.Direction.FORWARD);
    }

    public void setPower(double power){
        gate.setPosition(power);
    }

    public void stop(){
        gate.setPosition(0);
    }

    public double getPower(){
        return gate.getPosition();
    }


}
