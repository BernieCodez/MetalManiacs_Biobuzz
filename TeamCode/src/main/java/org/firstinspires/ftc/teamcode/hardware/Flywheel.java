package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Config.*;

public class Flywheel {

    private final DcMotorEx rightFlywheel;
    private final DcMotorEx leftFlywheel;

    private double optimalSpeed;

    //startup ramping variables
    private double commandedVelocity = 0;
    private long lastUpdateTime = 0;

    private boolean shootingPIDF = false;
    private boolean spinningUp = false;
    public double CURRENT_FLYWHEEL_KP;
    public Flywheel(HardwareMap hardwareMap) {

        rightFlywheel = hardwareMap.get(DcMotorEx.class, RIGHT_FLYWHEEL);
        leftFlywheel = hardwareMap.get(DcMotorEx.class, LEFT_FLYWHEEL);

        //set pidf vals
        applyStartupPIDF();

        rightFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setPower(0);

        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setPower(0);
    }

    private void applyStartupPIDF() {
        CURRENT_FLYWHEEL_KP = FLYWHEEL_STARTUP_KP;
        rightFlywheel.setVelocityPIDFCoefficients(
                FLYWHEEL_STARTUP_KP,
                FLYWHEEL_KI,
                FLYWHEEL_KD,
                FLYWHEEL_KF
        );

        leftFlywheel.setVelocityPIDFCoefficients(
                FLYWHEEL_STARTUP_KP,
                FLYWHEEL_KI,
                FLYWHEEL_KD,
                FLYWHEEL_KF
        );
    }

    private void applyShootingPIDF() {
        CURRENT_FLYWHEEL_KP = FLYWHEEL_SHOOTING_KP;
        rightFlywheel.setVelocityPIDFCoefficients(
                FLYWHEEL_SHOOTING_KP,
                FLYWHEEL_KI,
                FLYWHEEL_KD,
                FLYWHEEL_KF
        );

        leftFlywheel.setVelocityPIDFCoefficients(
                FLYWHEEL_SHOOTING_KP,
                FLYWHEEL_KI,
                FLYWHEEL_KD,
                FLYWHEEL_KF
        );
    }
    public void setVelocityByDistance(double distance){
        optimalSpeed = FLYWHEEL_DISTANCE_SLOPE * distance + FLYWHEEL_DISTANCE_INTERCEPT;
        setVelocity(optimalSpeed);
    }

    //set velocity in encoder ticks per second
    public void setVelocity(double velocity) {
        if (velocity <= 0) {
            stop();
            return;
        }

        //use startup ramp p val only when spinning up
        if (!spinningUp) {
            spinningUp = true;
            shootingPIDF = false;

            commandedVelocity = 0;
            lastUpdateTime = System.nanoTime();

            applyStartupPIDF();
        }

        optimalSpeed = velocity;
    }

    public void update() {
        if (!spinningUp) return;

        long currentTime = System.nanoTime();

        double deltaTime = (currentTime - lastUpdateTime) / 1_000_000_000.0;

        lastUpdateTime = currentTime;

        //ramp to target velo
        commandedVelocity = Math.min(commandedVelocity + FLYWHEEL_RAMP_RATE * deltaTime, optimalSpeed);

        rightFlywheel.setVelocity(commandedVelocity);
        leftFlywheel.setVelocity(commandedVelocity);

        //switch to aggressive pidf once within a good tolerance of the target velo
        if (!shootingPIDF && commandedVelocity >= optimalSpeed && getVelocity() >= optimalSpeed * 0.97) {
            applyShootingPIDF();
            shootingPIDF = true;
        }
    }

    public void stop() {
        rightFlywheel.setVelocity(0);
        leftFlywheel.setVelocity(0);

        optimalSpeed = 0;
        commandedVelocity = 0;

        spinningUp = false;
        shootingPIDF = false;

        applyStartupPIDF();
    }

    public double getVelocity() {return (rightFlywheel.getVelocity() + leftFlywheel.getVelocity()) / 2;}

    public void setPower(double power){
        rightFlywheel.setPower(power);
        leftFlywheel.setPower(power);
    }

    public double getPower() {return (rightFlywheel.getPower() + leftFlywheel.getPower()) / 2;}

    public boolean atTargetVelocity() {return Math.abs(getVelocity() - optimalSpeed) < FLYWHEEL_VELOCITY_TOLERANCE;}
    public double getLeftVelocity() {
        return leftFlywheel.getVelocity();
    }

    public double getRightVelocity() {
        return rightFlywheel.getVelocity();
    }

    public double getTargetVelocity() {
        return optimalSpeed;
    }
}