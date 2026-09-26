package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.Config.*;

public class Hood {
    private Servo hood;

    public Hood(HardwareMap hardwareMap){
        hood = hardwareMap.get(Servo.class, HOOD);
        hood.setDirection(Servo.Direction.FORWARD);
    }

    public void open(){
        hood.setPosition(HOOD_NECTAR);
    }

    public void close(){
        hood.setPosition(HOOD_POLLEN);
    }

    public double getPosition(){
        return hood.getPosition();
    }

}
