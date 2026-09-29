package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Config.*;

public class Flywheel {

    private final DcMotorEx rightFlywheel;
    private final DcMotorEx leftFlywheel;

    private double optimalSpeed;

    public Flywheel(HardwareMap hardwareMap) {

        rightFlywheel = hardwareMap.get(DcMotorEx.class, LEFT_FLYWHEEL);
        leftFlywheel = hardwareMap.get(DcMotorEx.class, RIGHT_FLYWHEEL);

        //set pidf vals
        rightFlywheel.setVelocityPIDFCoefficients(
                FLYWHEEL_KP,
                FLYWHEEL_KI,
                FLYWHEEL_KD,
                FLYWHEEL_KF
        );

        //set pidf vals
        leftFlywheel.setVelocityPIDFCoefficients(
                FLYWHEEL_KP,
                FLYWHEEL_KI,
                FLYWHEEL_KD,
                FLYWHEEL_KF
        );

        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setPower(0);

        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setPower(0);
    }

    public void setVelocityByDistance(double distance){
        optimalSpeed = FLYWHEEL_DISTANCE_SLOPE * distance + FLYWHEEL_DISTANCE_INTERCEPT;
        setVelocity(optimalSpeed);
    }

    //set velocity in encoder ticks per second
    public void setVelocity(double velocity) {
        rightFlywheel.setVelocity(velocity);
        leftFlywheel.setVelocity(velocity);
    }

    public void stop() {
        rightFlywheel.setVelocity(0);
        leftFlywheel.setVelocity(0);
    }
    public double getVelocity() {return (rightFlywheel.getVelocity() + leftFlywheel.getVelocity()) / 2;}

    public double getPower() {return (rightFlywheel.getPower() + leftFlywheel.getPower()) / 2;}

    public boolean atTargetVelocity() {return Math.abs(getVelocity() - optimalSpeed) < FLYWHEEL_VELOCITY_TOLERANCE;}
}