package org.firstinspires.ftc.teamcode.controllers;
import static org.firstinspires.ftc.teamcode.Config.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.FlowerIntake;
import org.firstinspires.ftc.teamcode.hardware.IntakeTransfer;
public class IntakeTransferController {
    IntakeTransfer intakeTransfer;
    FlowerIntake flowerIntake;
    public double targetPower = 0;
    public double currentPower = 0;
    private long lastUpdateTime = 0;


    public IntakeTransferController(HardwareMap hardwareMap){
        intakeTransfer = new IntakeTransfer(hardwareMap);
        flowerIntake = new FlowerIntake(hardwareMap);
        lastUpdateTime = System.nanoTime();
    }

    public void update(){
        long currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastUpdateTime) / 1_000_000_000.0;
        lastUpdateTime = currentTime;

        if (intakeTransfer.isJamming()){ //stop the motor if jams
            stop();
        }

        double maxChange = RAMP_RATE * deltaTime;
        double difference = targetPower - currentPower;

        if (Math.abs(difference) <= maxChange) {
            currentPower = targetPower; //maintain speed because already there
        } else {
            currentPower += Math.signum(difference) * maxChange; //ramp if not there
        }
        intakeTransfer.setPower(currentPower);

        //update flower wedge
        flowerIntake.update();
    }

    public double getCurrent(){return intakeTransfer.getCurrent();}

    public void toggleForward(){
        if (intakeTransfer.getPower() <= 0){ //if its in reverse cut immediately to forward
            targetPower = MAX_INTAKE_TRANSFER_POWER;
        } else {
            targetPower = 0;
        }
    }

    public void toggleReverse(){
        if (intakeTransfer.getPower() >= 0){
            targetPower = -MAX_INTAKE_TRANSFER_POWER;
        } else {
            targetPower = 0;
        }
    }

    public void stop(){targetPower = 0;}
    public void setPower(double power){
        targetPower = power;
    }
    public boolean isOn(){return Math.abs(intakeTransfer.getPower())>0;}
    public double getPower(){return intakeTransfer.getPower();}
    public void toggleFlowerIntake(){
        flowerIntake.toggle();
    }

    public void flowerIntakeDown(){
        flowerIntake.down();
    }

    public void flowerIntakeUp(){
        flowerIntake.up();
    }
}
