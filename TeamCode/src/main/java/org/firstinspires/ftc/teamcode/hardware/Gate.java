package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config;

public class Gate {
    private Servo gate;

    public Gate(HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, Config.GATE);
        gate.setDirection(Servo.Direction.FORWARD);
    }

    public void open(){
        gate.setPosition(Config.GATE_OPEN);
    }

    public void close(){
        gate.setPosition(Config.GATE_CLOSE);
    }

    public double getPosition(){
        return gate.getPosition();
    }


}
