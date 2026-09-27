package org.firstinspires.ftc.teamcode.controllers;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Config.*;

import org.firstinspires.ftc.teamcode.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Gate;
import org.firstinspires.ftc.teamcode.hardware.Hood;

public class OuttakeController {
    private Flywheel flywheel;
    private Gate gate;
    private Hood hood;
    private ColorSensor colorSensor;

    public String[] elements;
    public java.util.ArrayList<String> elementList = new java.util.ArrayList<>();

    private Element lastTrackedDetection = Element.NONE;
    private final ElapsedTime confirmationTimer = new ElapsedTime();
    private boolean hasAddedCurrentElement = false;

    public OuttakeController(HardwareMap hardwareMap){
        flywheel = new Flywheel(hardwareMap);
        gate = new Gate(hardwareMap);
        hood = new Hood(hardwareMap);
        colorSensor = new ColorSensor(hardwareMap);
        elements = new String[0];
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
            String elementName = "";
            if (currentDetection == Element.RED_NECTAR) elementName = "Red Nectar";
            else if (currentDetection == Element.BLUE_NECTAR) elementName = "Blue Nectar";
            else if (currentDetection == Element.POLLEN) elementName = "Pollen";

            elementList.add(elementName);
            elements = elementList.toArray(new String[0]);
            hasAddedCurrentElement = true;
        }
    }

    public void fire(){
        gate.open();
        if (!elementList.isEmpty()) {
            elementList.remove(0);
            elements = elementList.toArray(new String[0]);
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