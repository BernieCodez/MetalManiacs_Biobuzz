
package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Config.*;

public class FlowerIntake {

    private Servo rightWedge;
    private Servo leftWedge;
    public FlowerIntake(HardwareMap hardwareMap) {
        rightWedge = hardwareMap.get(Servo.class, RIGHT_WEDGE);
        leftWedge = hardwareMap.get(Servo.class, LEFT_WEDGE);
    }

    public void toggle(){
        rightWedge.setPosition(rightWedge.getPosition() == WEDGE_DOWN ? WEDGE_UP : WEDGE_DOWN);
        leftWedge.setPosition(leftWedge.getPosition() == WEDGE_DOWN ? WEDGE_UP : WEDGE_DOWN);
    }
}