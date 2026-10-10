
package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FlowerIntake {

    private Servo rightWedge;
    private Servo leftWedge;
    public double openPosition = 1;
    public double closePosition = 0;
    public FlowerIntake(HardwareMap hardwareMap) {
        rightWedge = hardwareMap.get(Servo.class, "rightWedge");
        leftWedge = hardwareMap.get(Servo.class, "leftWedge");
    }

    public void toggle(){
        rightWedge.setPosition(rightWedge.getPosition() == closePosition ? openPosition : closePosition);
        leftWedge.setPosition(leftWedge.getPosition() == closePosition ? openPosition : closePosition);
    }
}