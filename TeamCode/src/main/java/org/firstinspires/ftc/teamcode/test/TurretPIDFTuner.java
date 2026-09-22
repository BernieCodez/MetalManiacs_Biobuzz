package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Config;

//HOW TO TUNE
//Tune POSITION_P first then do all the velocity pids later

//v uncommenting this will hide it on the driver station
//@Disabled
@Configurable
@TeleOp(name = "Turret PIDF Tuner", group = "A - Test")
public class TurretPIDFTuner extends LinearOpMode {

    public static int TARGET_POSITION = 0;

    //position pid (only need p for position), top layer
    public static double POSITION_P = 0;

    //velocity pidf, bottom layer controlled by position pid
    public static double VELOCITY_P = 0;
    public static double VELOCITY_I = 0;
    public static double VELOCITY_D = 0;
    public static double VELOCITY_F = 0;


    //position tolerance!
    public static int POSITION_TOLERANCE = 10;

    public static boolean RUN_TURRET = false; //enable to run

    @Override
    public void runOpMode() {

        DcMotorEx turret = hardwareMap.get(DcMotorEx.class, Config.TURRET);

        //init velocity pidf first because position pid relies on it
        turret.setVelocityPIDFCoefficients(
                VELOCITY_P,
                VELOCITY_I,
                VELOCITY_D,
                VELOCITY_F
        );

        //init position pid after
        turret.setPositionPIDFCoefficients(
                POSITION_P
        );

        turret.setTargetPositionTolerance(POSITION_TOLERANCE);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addLine("Turret PIDF Tuner");
        telemetry.addLine("----------------------");
        telemetry.addLine("Panels controls the PIDF values.");
        telemetry.addLine("Set RUN_TURRET = true to move.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (Config.TUNING_ENABLED){
                //update velocity pidf first
                turret.setVelocityPIDFCoefficients(
                        Config.TURRET_VELOCITY_P,
                        Config.TURRET_VELOCITY_I,
                        Config.TURRET_VELOCITY_D,
                        Config.TURRET_VELOCITY_F
                );

                //update position pid second
                turret.setPositionPIDFCoefficients(Config.TURRET_POSITION_P);

                //update tolerance
                turret.setTargetPositionTolerance(Config.TURRET_POSITION_TOLERANCE);
            }else{
                //update velocity pidf first
                turret.setVelocityPIDFCoefficients(
                        VELOCITY_P,
                        VELOCITY_I,
                        VELOCITY_D,
                        VELOCITY_F
                );

                //update position pid second
                turret.setPositionPIDFCoefficients(POSITION_P);

                //update tolerance
                turret.setTargetPositionTolerance(POSITION_TOLERANCE);
            }

            if (RUN_TURRET) {
                turret.setTargetPosition(TARGET_POSITION);
                turret.setPower(1.0);
            } else {
                turret.setPower(0);
                turret.setTargetPosition(turret.getCurrentPosition());
            }

            int currentPosition = turret.getCurrentPosition();

            int error = TARGET_POSITION - currentPosition;

            //telemetry
            telemetry.addData("Target Position", TARGET_POSITION);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("At Target", turret.isBusy() ? "NO" : "YES");
            telemetry.addLine("");
            telemetry.addData("Position P", POSITION_P);
            telemetry.addLine("");
            telemetry.addData("Velocity P", VELOCITY_P);
            telemetry.addData("Velocity I", VELOCITY_I);
            telemetry.addData("Velocity D", VELOCITY_D);
            telemetry.addData("Velocity F", VELOCITY_F);
            telemetry.addLine("");
            telemetry.addData("Running", RUN_TURRET);
            telemetry.update();
        }

        turret.setPower(0);
    }
}