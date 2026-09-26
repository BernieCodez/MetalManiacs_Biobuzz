package org.firstinspires.ftc.teamcode.controllers;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.hardware.Limelight;
import org.firstinspires.ftc.teamcode.hardware.Turret;

public class AutoAimController {

    private final Limelight limelight;
    private final Turret turret;
    private Config.Hive activeHive = null;
    private double targetAngle; //MEASURED IN DEGREES :)
    private final double MANUAL_TURRET_SPEED = 30; //multiplier

    public AutoAimController(HardwareMap hardwareMap) {
        limelight = new Limelight(hardwareMap);
        turret = new Turret(hardwareMap);

    }

    public void update(boolean shouldAutoAim, String teamColor, Pose robot, double manual) {
        //update active target
        activeHive = getTargetHive(teamColor, robot);

        //fetch limelight tag data
        limelight.update(activeHive);

        if (!shouldAutoAim) {
            //Put manual aim stuff in here
            targetAngle = turret.getHeading();
            targetAngle += manual*MANUAL_TURRET_SPEED;
        }else{
            targetAngle = calculateLocalizedAngle(robot); //calc using pedro pathing localization

            //april tag is visible
            if (limelight.tVisible) {
                //correct calculated localized target angle with limelight calculated angle
                targetAngle += limelight.tx; //-= IF THE TURRET ENDS UP SPINNING THE OPPOSITE WAY WHEN IT SEES AN APRIL TAG
            }
        }

        //tell the turret where to go
        turret.update(normalizeAngle(targetAngle));
    }

    private Config.Hive getTargetHive(String teamColor, Pose robot) {
        boolean top = robot.y() > Config.HIVE_BOUNDARY;

        if (teamColor.equals("red")) {
            return top ? Config.Hive.RED_TOP : Config.Hive.RED_BOTTOM;
        } else {
            return top ? Config.Hive.BLUE_TOP : Config.Hive.BLUE_BOTTOM;
        }
    }

    //calculates the amount the turret needs to rotate in order to be facing the goal from its localized pedro position
    public double calculateLocalizedAngle(Pose robot) {
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