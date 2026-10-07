package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.teamcode.Config.*;

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

        DcMotorEx rightFlywheel = hardwareMap.get(DcMotorEx.class, RIGHT_FLYWHEEL);
        DcMotorEx leftFlywheel = hardwareMap.get(DcMotorEx.class, LEFT_FLYWHEEL);

        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setVelocityPIDFCoefficients(
                KP,
                KI,
                KD,
                KF
        );
        leftFlywheel.setVelocityPIDFCoefficients(
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
            if (TUNING_ENABLED){
                rightFlywheel.setVelocityPIDFCoefficients(
                        FLYWHEEL_KP,
                        FLYWHEEL_KI,
                        FLYWHEEL_KD,
                        FLYWHEEL_KF
                );
                leftFlywheel.setVelocityPIDFCoefficients(
                        FLYWHEEL_KP,
                        FLYWHEEL_KI,
                        FLYWHEEL_KD,
                        FLYWHEEL_KF
                );
            }else{
                rightFlywheel.setVelocityPIDFCoefficients(
                        KP,
                        KI,
                        KD,
                        KF
                );
            }
            leftFlywheel.setVelocityPIDFCoefficients(
                    KP,
                    KI,
                    KD,
                    KF
            );


            if (RUN_FLYWHEEL) {
                rightFlywheel.setVelocity(TARGET_VELOCITY);
                leftFlywheel.setVelocity(TARGET_VELOCITY);
            } else {
                rightFlywheel.setVelocity(0);
                leftFlywheel.setVelocity(0);
            }

            double currentVelocity = rightFlywheel.getVelocity();

            double error = TARGET_VELOCITY - currentVelocity;

            //telemetry
            telemetry.addData("Target Velocity", "%.2f ticks/s", TARGET_VELOCITY);
            telemetry.addData("Current Velocity", "%.2f ticks/s", currentVelocity);
            telemetry.addData("Error", "%.2f ticks/s", error);
            telemetry.addData("Power", "%.3f", rightFlywheel.getPower());
            telemetry.addLine("");
            telemetry.addData("kP", KP);
            telemetry.addData("kI", KI);
            telemetry.addData("kD", KD);
            telemetry.addData("kF", KF);
            telemetry.addLine("");
            telemetry.addData("Running", RUN_FLYWHEEL);

            telemetry.update();
        }

        rightFlywheel.setVelocity(0);
        leftFlywheel.setVelocity(0);
    }
}
