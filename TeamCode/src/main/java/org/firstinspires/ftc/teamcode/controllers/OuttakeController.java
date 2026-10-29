package org.firstinspires.ftc.teamcode.controllers;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Config.*;

import org.firstinspires.ftc.teamcode.deprecated.ColorSensor;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.NectarGate;
import org.firstinspires.ftc.teamcode.hardware.PollenGate;

import java.util.ArrayList;
public class OuttakeController {
    public Flywheel flywheel;
    private PollenGate pollenGate;
    private NectarGate nectarGate;
    private ColorSensor colorSensor;

    public ArrayList<Element> elementList = new ArrayList<>();

    private Element lastTrackedDetection = Element.NONE;
    private final ElapsedTime confirmationTimer = new ElapsedTime();
    private boolean hasAddedCurrentElement = false;
    public boolean tomuchballs = false;
    public boolean useFlywheel = false;
    public double distance = 0;

    public OuttakeController(HardwareMap hardwareMap){
        flywheel = new Flywheel(hardwareMap);
        pollenGate = new PollenGate(hardwareMap);
        nectarGate = new NectarGate(hardwareMap);
        colorSensor = new ColorSensor(hardwareMap);
    }

    public Element getRealTimeDetection() {
        return colorSensor.getDetectedColor();
    }

    public void update(Alliance.Hive activeHive, Pose robot){
        if (activeHive == null) {
            flywheel.stop();
            useFlywheel = false;
        } else {
            distance = Math.hypot(robot.x() - activeHive.pose.x(), robot.y() - activeHive.pose.y());
            flywheel.setVelocityByDistance(distance);
            useFlywheel = true;
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
        pollenGate.open();
        if (!elementList.isEmpty()) {
            elementList.remove(0);
        }
    }

    public void fireNectar(){
        nectarGate.open();
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
        pollenGate.close();
        nectarGate.close();
    }

    public void stop(){
        flywheel.stop();
        pollenGate.open();
        nectarGate.open();
    }
}