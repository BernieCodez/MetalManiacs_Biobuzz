package org.firstinspires.ftc.teamcode.controllers;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Config.*;

import org.firstinspires.ftc.teamcode.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Gate;
import org.firstinspires.ftc.teamcode.hardware.Hood;

import java.util.ArrayList;
public class OuttakeController {
    private Flywheel flywheel;
    private Gate gate;
    private Hood hood;
    private ColorSensor colorSensor;

    public ArrayList<Element> elementList = new ArrayList<>();

    private Element lastTrackedDetection = Element.NONE;
    private final ElapsedTime confirmationTimer = new ElapsedTime();
    private boolean hasAddedCurrentElement = false;
    public boolean tomuchballs = false;

    public OuttakeController(HardwareMap hardwareMap){
        flywheel = new Flywheel(hardwareMap);
        gate = new Gate(hardwareMap);
        hood = new Hood(hardwareMap);
        colorSensor = new ColorSensor(hardwareMap);
    }

    public Element getRealTimeDetection() {
        return colorSensor.getDetectedColor();
    }

    public void update(Alliance.Hive activeHive, Pose robot){
        if (activeHive == null) {
            flywheel.stop();
        } else {
            double distance = Math.hypot(robot.x() - activeHive.pose.x(), robot.y() - activeHive.pose.y());
            flywheel.setVelocityByDistance(distance);
        }

        Element currentDetection = colorSensor.getDetectedColor();

        if (currentDetection != lastTrackedDetection) {
            lastTrackedDetection = currentDetection;
            confirmationTimer.reset();
            hasAddedCurrentElement = false;
        }

        if (currentDetection != Element.NONE && !hasAddedCurrentElement && confirmationTimer.seconds() >= 0.1) {
            elementList.add(currentDetection);
            hasAddedCurrentElement = true;
        }
    }

    public void fire(){
        gate.open();
        if (!elementList.isEmpty()) {
            elementList.remove(0);
        }
    }

    public void safelimit(){
        if (elementList.size() > 3){
            tomuchballs = true;
        }
    }

    public void close(){
        gate.close();
    }

    public void stop(){
        flywheel.stop();
        gate.open();
    }
}