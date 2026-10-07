package org.firstinspires.ftc.teamcode.controllers;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Config.*;
import org.firstinspires.ftc.teamcode.hardware.Limelight;
public class AutoAimController {

    private final Limelight limelight;
    private double targetHeading;
    private double headingError;
    public double rotation = 0;

    public AutoAimController(HardwareMap hardwareMap) {
        limelight = new Limelight(hardwareMap);

    }

    public void update(Alliance.Hive activeHive, Pose robot) {
        //fetch limelight tag data
        limelight.update(activeHive);

        targetHeading = calculateLocalizedHeading(activeHive, robot); //calc using pedro pathing localization

        headingError = normalizeAngle(targetHeading - Math.toDegrees(robot.heading()));

        //april tag is visible
        if (limelight.tVisible) {
            //correct calculated localized target angle with limelight calculated angle
            headingError = normalizeAngle(headingError + limelight.tx); //USE - IF THE ROBOT ENDS UP SPINNING THE OPPOSITE WAY WHEN IT SEES AN APRIL TAG
        }

        //tell the robot body where to go
        rotation = clip(AUTOAIM_P * headingError, -AUTOAIM_MAX_ROTATION_SPEED, AUTOAIM_MAX_ROTATION_SPEED);
    }

    //calculates the amount the robot needs to rotate in order to be facing the goal from its localized pedro position
    public double calculateLocalizedHeading(Alliance.Hive activeHive, Pose robot) {
        double deltaX = activeHive.pose.x() - robot.x();
        double deltaY = activeHive.pose.y() - robot.y();

        return  Math.toDegrees(Math.atan2(deltaY, deltaX));
    }

    private double normalizeAngle(double angle) {//normalizes from -180 to 180
        return Math.toDegrees(
                Math.atan2(
                        Math.sin(Math.toRadians(angle)),
                        Math.cos(Math.toRadians(angle))
                )
        );
    }

    private double clip(double value, double min, double max){return Math.max(min, Math.min(value, max));}
}