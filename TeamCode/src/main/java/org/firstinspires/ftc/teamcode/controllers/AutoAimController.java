package org.firstinspires.ftc.teamcode.controllers;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.hardware.Limelight;
import org.firstinspires.ftc.teamcode.hardware.Turret;

public class AutoAimController {

    private final Limelight limelight;
    private final Turret turret;
    private double targetAngle; //MEASURED IN DEGREES :)
    private final double MANUAL_TURRET_SPEED = 30; //multiplier

    public AutoAimController(HardwareMap hardwareMap) {
        limelight = new Limelight(hardwareMap);
        turret = new Turret(hardwareMap);

    }

    public void update(Config.Hive activeHive, Pose robot, boolean shouldAutoAim, double manual) {
        //fetch limelight tag data
        limelight.update(activeHive);

        if (!shouldAutoAim) {
            //Put manual aim stuff in here
            targetAngle = turret.getHeading();
            targetAngle += manual*MANUAL_TURRET_SPEED;
        }else{
            targetAngle = calculateLocalizedAngle(activeHive, robot); //calc using pedro pathing localization

            //april tag is visible
            if (limelight.tVisible) {
                //correct calculated localized target angle with limelight calculated angle
                targetAngle += limelight.tx; //-= IF THE TURRET ENDS UP SPINNING THE OPPOSITE WAY WHEN IT SEES AN APRIL TAG
            }
        }

        //tell the turret where to go
        turret.update(normalizeAngle(targetAngle));
    }

    //calculates the amount the turret needs to rotate in order to be facing the goal from its localized pedro position
    public double calculateLocalizedAngle(Config.Hive activeHive, Pose robot) {
        double deltaX = activeHive.pose.x() - robot.x();
        double deltaY = activeHive.pose.y() - robot.y();

        double targetHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));

        double desiredTurretHeading = targetHeading - robot.heading();

        return normalizeAngle(desiredTurretHeading);
    }

    private double normalizeAngle(double angle) {//normalizes from -180 to 180
        return Math.toDegrees(
                Math.atan2(
                        Math.sin(Math.toRadians(angle)),
                        Math.cos(Math.toRadians(angle))
                )
        );
    }
}