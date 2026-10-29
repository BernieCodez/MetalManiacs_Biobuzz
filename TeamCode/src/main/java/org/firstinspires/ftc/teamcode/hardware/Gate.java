package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Config.*;

public class Gate {
    private Servo nectarGate;
    private Servo pollenGate;

    public Gate(HardwareMap hardwareMap){
        nectarGate = hardwareMap.get(Servo.class, NECTAR_GATE);
        nectarGate.setDirection(Servo.Direction.FORWARD);
        pollenGate = hardwareMap.get(Servo.class, POLLEN_GATE);
        pollenGate.setDirection(Servo.Direction.FORWARD);
    }

    //combined servo code
    public void open(){
        openNectar();
        openPollen();
    }
    public void close(){
        closeNectar();
        closePollen();
    }

    //nectar servo code
    public void openNectar(){
        nectarGate.setPosition(NECTAR_GATE_OPEN);
    }

    public void closeNectar(){
        nectarGate.setPosition(NECTAR_GATE_CLOSE);
    }

    public double getNectarPosition(){
        return nectarGate.getPosition();
    }

    //pollen servo code
    public void openPollen(){pollenGate.setPosition(POLLEN_GATE_OPEN);}

    public void closePollen(){
        pollenGate.setPosition(POLLEN_GATE_CLOSE);
    }

    public double getPollenPosition(){return pollenGate.getPosition();}


}
