package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Config;

//v uncommenting this will hide it on the driver station
//@Disabled
@Configurable
@TeleOp(name = "Flywheel PIDF Tuner", group = "A - Test")
public class FlywheelPIDFTuner extends LinearOpMode {


    public static double TARGET_VELOCITY = 0;

    //velocity pidf
    public static double KP = 0;
    public static double KI = 0;
    public static double KD = 0;
    public static double KF = 0;
    public static final double VELOCITY_TOLERANCE = 10;

    public static boolean RUN_FLYWHEEL = false; //enable to run

    @Override
    public void runOpMode() {

        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, Config.FLYWHEEL);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setVelocityPIDFCoefficients(
                KP,
                KI,
                KD,
                KF
        );

        telemetry.addLine("Flywheel PIDF Tuner");
        telemetry.addLine("----------------------");
        telemetry.addLine("Panels controls the PIDF values.");
        telemetry.addLine("Set RUN_FLYWHEEL = true to spin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (Config.TUNING_ENABLED){
                flywheel.setVelocityPIDFCoefficients(
                        Config.FLYWHEEL_KP,
                        Config.FLYWHEEL_KI,
                        Config.FLYWHEEL_KD,
                        Config.FLYWHEEL_KF
                );
            }else{
                flywheel.setVelocityPIDFCoefficients(
                        KP,
                        KI,
                        KD,
                        KF
                );
            }


            if (RUN_FLYWHEEL) {
                flywheel.setVelocity(TARGET_VELOCITY);
            } else {
                flywheel.setVelocity(0);
            }

            double currentVelocity = flywheel.getVelocity();

            double error = TARGET_VELOCITY - currentVelocity;

            //telemetry
            telemetry.addData("Target Velocity", "%.2f ticks/s", TARGET_VELOCITY);
            telemetry.addData("Current Velocity", "%.2f ticks/s", currentVelocity);
            telemetry.addData("Error", "%.2f ticks/s", error);
            telemetry.addData("Power", "%.3f", flywheel.getPower());
            telemetry.addLine("");
            telemetry.addData("kP", KP);
            telemetry.addData("kI", KI);
            telemetry.addData("kD", KD);
            telemetry.addData("kF", KF);
            telemetry.addLine("");
            telemetry.addData("Running", RUN_FLYWHEEL);

            telemetry.update();
        }

        flywheel.setVelocity(0);
    }
}