package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Config.*;

public class PollenGate {
    private Servo pollenGate;

    public PollenGate(HardwareMap hardwareMap){
        pollenGate = hardwareMap.get(Servo.class, POLLEN_GATE);
        pollenGate.setDirection(Servo.Direction.FORWARD);
    }

    public void open(){
        pollenGate.setPosition(POLLEN_GATE_OPEN);
    }

    public void close(){
        pollenGate.setPosition(POLLEN_GATE_CLOSE);
    }

    public double getPosition(){
        return pollenGate.getPosition();
    }


}
