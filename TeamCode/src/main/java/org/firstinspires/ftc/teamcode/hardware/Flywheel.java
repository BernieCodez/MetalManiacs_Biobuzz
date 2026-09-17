package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Config;

public class Flywheel {
    private DcMotorEx flywheel;

    double p = 10.0;
    double i = 1.0;
    double d = 5.0;
    double f = 15;

    public Flywheel(HardwareMap hardwareMap){
        flywheel = hardwareMap.get(DcMotorEx.class, Config.FLYWHEEL);
        flywheel.setDirection(DcMotorEx.Direction.FORWARD);

        //Reset the motor encoders on start
        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, i, d, f);
        flywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void setFlywheelVelocity(double ticksPerSecond){
        flywheel.setVelocity(ticksPerSecond);
    }

    public double getFlywheelVelocity(){
        return flywheel.getVelocity();
    }
}
