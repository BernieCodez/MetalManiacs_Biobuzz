package org.firstinspires.ftc.teamcode.controllers;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Gate;

public class OuttakeController {
    private Flywheel flywheel;
    private Gate gate;
    public OuttakeController(HardwareMap hardwareMap){
        flywheel = new Flywheel(hardwareMap);
        gate = new Gate(hardwareMap);
    }
    public void update(Config.Hive activeHive, Pose robot){ //maintain flywheel velocity
        if (activeHive == null) {
            flywheel.stop();
            return;
        }
        double distance = Math.hypot(robot.x() - activeHive.pose.x(), robot.y() - activeHive.pose.y());
        flywheel.setVelocityByDistance(distance);
    }

    public void fire(){ //feed balls
        gate.open();
    }

    public void close(){ //stop feeding
        gate.close();
    }

    public void stop(){
        flywheel.stop();
        gate.open();
    }
}
