package org.firstinspires.ftc.teamcode.controllers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Config.*;

import org.firstinspires.ftc.teamcode.deprecated.ColorSensor;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Gate;

import java.util.ArrayList;

@Configurable
public class OuttakeController {
    public static double TELEOP_FLYWHEEL_VELOCITY = 1160;
    public static double TELEOP_SHOT_OPEN_MS = 1400;
    public static double TELEOP_SHOT_GAP_MS = 400;
    public static int TELEOP_SHOTS_PER_PRESS = 1;
    public static boolean TELEOP_NECTAR_GATE_ONLY = false;
    public static boolean TELEOP_RAMP_ENABLED = true;
    public static double TELEOP_RAMP_START_MS = 30;
    public static double TELEOP_RAMP_DURATION_MS = 5000;
    public static double TELEOP_RAMP_AMOUNT = 300;

    public enum ShotPhase { IDLE, OPEN, GAP }

    public ShotPhase shotPhase = ShotPhase.IDLE;
    private int shotsLeft = 0;
    private final ElapsedTime shotTimer = new ElapsedTime();
    public double targetVelocity = 0;

    public Flywheel flywheel;
    private Gate gate;
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
        gate = new Gate(hardwareMap);
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
//            flywheel.setVelocityByDistance(distance);
            flywheel.setVelocity(1127); //was 1125
//            flywheel.setVelocity(Math.min(1300, FLYWHEEL_DISTANCE_INTERCEPT + FLYWHEEL_DISTANCE_SLOPE * distance));
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

        flywheel.update();
    }

    public void updateTeleop(Alliance.Hive activeHive, Pose robot){
        if (activeHive != null) {
            distance = Math.hypot(robot.x() - activeHive.pose.x(), robot.y() - activeHive.pose.y());
        }
        stepBurst();
        targetVelocity = teleopTargetVelocity();
        flywheel.setVelocity(targetVelocity);
        useFlywheel = true;
        flywheel.update();
    }

    public void startBurst(){
        if (shotPhase != ShotPhase.IDLE) return;
        shotsLeft = Math.max(1, TELEOP_SHOTS_PER_PRESS);
        openBurstGate();
    }

    public void cancelBurst(){
        gate.close();
        shotPhase = ShotPhase.IDLE;
        shotsLeft = 0;
    }

    public boolean isFiring(){return shotPhase != ShotPhase.IDLE;}

    private void openBurstGate(){
        if (TELEOP_NECTAR_GATE_ONLY) fireNectar();
        else fire();
        shotPhase = ShotPhase.OPEN;
        shotTimer.reset();
    }

    private void stepBurst(){
        if (shotPhase == ShotPhase.OPEN && shotTimer.milliseconds() >= TELEOP_SHOT_OPEN_MS) {
            gate.close();
            shotPhase = ShotPhase.GAP;
            shotTimer.reset();
        } else if (shotPhase == ShotPhase.GAP && shotTimer.milliseconds() >= TELEOP_SHOT_GAP_MS) {
            shotsLeft--;
            if (shotsLeft > 0) openBurstGate();
            else shotPhase = ShotPhase.IDLE;
        }
    }

    private double teleopTargetVelocity(){
        if (!TELEOP_RAMP_ENABLED || shotPhase != ShotPhase.OPEN) return TELEOP_FLYWHEEL_VELOCITY;
        double msIntoRamp = shotTimer.milliseconds() - TELEOP_RAMP_START_MS;
        if (msIntoRamp <= 0) return TELEOP_FLYWHEEL_VELOCITY;
        double fraction = 1;
        if (TELEOP_RAMP_DURATION_MS > 0) fraction = Math.min(1, msIntoRamp / TELEOP_RAMP_DURATION_MS);
        return TELEOP_FLYWHEEL_VELOCITY + TELEOP_RAMP_AMOUNT * fraction;
    }

    public void fire(){
        gate.open();
        if (!elementList.isEmpty()) {
            elementList.remove(0);
        }
    }

    public void fireNectar(){
        gate.openNectar();
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

    public double getCurrent(){return flywheel.getCurrent();}
}