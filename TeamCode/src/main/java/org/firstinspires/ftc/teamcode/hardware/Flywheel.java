package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Config.*;

public class Flywheel {

    private final DcMotorEx flywheel;

    private double optimalSpeed;

    public Flywheel(HardwareMap hardwareMap) {

        flywheel = hardwareMap.get(DcMotorEx.class, FLYWHEEL);

        //set pidf vals
        flywheel.setVelocityPIDFCoefficients(
                FLYWHEEL_KP,
                FLYWHEEL_KI,
                FLYWHEEL_KD,
                FLYWHEEL_KF
        );

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel.setPower(0);
    }

    public void setVelocityByDistance(double distance){
        optimalSpeed = FLYWHEEL_DISTANCE_SLOPE * distance + FLYWHEEL_DISTANCE_INTERCEPT;
        flywheel.setVelocity(optimalSpeed);
    }

    //set velocity in encoder ticks per second
    public void setVelocity(double velocity) {
        flywheel.setVelocity(velocity);
    }

    public void stop() {
        flywheel.setVelocity(0);
    }

    public double getVelocity() {
        return flywheel.getVelocity();
    }

    public double getPower() {
        return flywheel.getPower();
    }

    public boolean atTargetVelocity() {
        return Math.abs(flywheel.getVelocity() - optimalSpeed) < FLYWHEEL_VELOCITY_TOLERANCE;
    }
}