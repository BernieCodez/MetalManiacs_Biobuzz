
package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.teamcode.Config.*;

public class FlowerIntake {

    private Servo rightWedge;
    private Servo leftWedge;
    private boolean wedgeUp = true;
    public FlowerIntake(HardwareMap hardwareMap) {
        rightWedge = hardwareMap.get(Servo.class, RIGHT_WEDGE);
        leftWedge = hardwareMap.get(Servo.class, LEFT_WEDGE);
    }

    public void down(){wedgeUp = false;}

    public void up(){wedgeUp = true;}

    public void toggle(){
        wedgeUp = !wedgeUp;
    }
    public void update(){
        rightWedge.setPosition(wedgeUp ? RIGHT_WEDGE_UP : RIGHT_WEDGE_DOWN);
        leftWedge.setPosition(wedgeUp ? LEFT_WEDGE_UP : LEFT_WEDGE_DOWN);
    }
}