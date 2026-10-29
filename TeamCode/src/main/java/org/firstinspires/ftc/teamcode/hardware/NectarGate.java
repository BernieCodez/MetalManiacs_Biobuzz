package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Config.*;

public class NectarGate {
    private Servo nectarGate;

    public NectarGate(HardwareMap hardwareMap){
        nectarGate = hardwareMap.get(Servo.class, NECTAR_GATE);
        nectarGate.setDirection(Servo.Direction.FORWARD);
    }

    public void open(){
        nectarGate.setPosition(NECTAR_GATE_OPEN);
    }

    public void close(){
        nectarGate.setPosition(NECTAR_GATE_CLOSE);
    }

    public double getPosition(){
        return nectarGate.getPosition();
    }


}
