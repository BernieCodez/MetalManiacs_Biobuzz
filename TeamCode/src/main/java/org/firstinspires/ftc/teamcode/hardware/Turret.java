package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config;

public class Turret {

    private final DcMotorEx turret;

    public Turret(HardwareMap hardwareMap) {

        turret = hardwareMap.get(DcMotorEx.class, Config.TURRET);

        // Apply the tuned velocity PIDF
        turret.setVelocityPIDFCoefficients(
                Config.TURRET_VELOCITY_P,
                Config.TURRET_VELOCITY_I,
                Config.TURRET_VELOCITY_D,
                Config.TURRET_VELOCITY_F
        );

        // Apply the tuned position P
        turret.setPositionPIDFCoefficients(Config.TURRET_POSITION_P);

        // Position tolerance
        turret.setTargetPositionTolerance(Config.TURRET_POSITION_TOLERANCE);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setPower(0);
    }

    public void update(double targetAngle){
        // Convert angle → encoder ticks
        int targetPosition = angleToTicks(targetAngle);

        // Command motor
        setPosition(targetPosition);
    }

    public int angleToTicks(double angle){
        return (int) Math.round(Config.TURRET_CENTER_TICKS + angle * Config.TURRET_TICKS_PER_DEGREE);
    }

    //uses encoder position!
    public void setPosition(int position) {

        position = Math.max(Config.MIN_TURRET_POSITION, Math.min(Config.MAX_TURRET_POSITION, position));//software limits :D

        turret.setTargetPosition(position);
        turret.setPower(1.0);
    }

    public void stop() {
        turret.setPower(0);
    }

    public int getPosition() {
        return turret.getCurrentPosition();
    }

    public int getTargetPosition() {
        return turret.getTargetPosition();
    }

    public int getError() {
        return getTargetPosition() - getPosition();
    }

    public boolean atTarget() {
        return Math.abs(getError())
                <= Config.TURRET_POSITION_TOLERANCE;
    }

    //returns if the turret is currently moving towards a position
    public boolean isBusy() {
        return turret.isBusy();
    }

    public double getPower() {
        return turret.getPower();
    }

    //resets the encoder but it actually needs to be at the correct spot when resetting
    public void resetEncoder() {
        turret.setPower(0);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getHeading() {
        return (turret.getCurrentPosition() - Config.TURRET_CENTER_TICKS) / Config.TURRET_TICKS_PER_DEGREE;
    }
}