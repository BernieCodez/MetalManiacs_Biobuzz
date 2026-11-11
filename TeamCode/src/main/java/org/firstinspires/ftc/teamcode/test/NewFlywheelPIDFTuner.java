package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.panels.Panels;
import com.bylazar.telemetry.Plugin;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static org.firstinspires.ftc.teamcode.Config.*;
import static org.firstinspires.ftc.teamcode.Config.FLYWHEEL_KD;
import static org.firstinspires.ftc.teamcode.Config.FLYWHEEL_KF;

import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Gate;
import org.firstinspires.ftc.teamcode.hardware.IntakeTransfer;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

//v uncommenting this will hide it on the driver station
//@Disabled
@Configurable
@TeleOp(name = "Flywheel Class PIDF Tuner", group = "A - Test")
public class NewFlywheelPIDFTuner extends LinearOpMode {
    public static double TARGET_VELOCITY = 0;

    public static boolean RUN_FLYWHEEL = false; //enable to run
    private final TelemetryManager panelsTelemetry =
            Plugin.INSTANCE.getManager();
    public IntakeTransfer intakeTransfer;
    public Flywheel flywheel;
    public Gate gate;
    public RumbleGamepad driver;

    @Override
    public void runOpMode() {

        flywheel = new Flywheel(hardwareMap);
        gate = new Gate(hardwareMap);
        driver = new RumbleGamepad(gamepad1);
        intakeTransfer = new IntakeTransfer(hardwareMap);

        telemetry.addLine("Flywheel PIDF Tuner");
        telemetry.addLine("----------------------");
        telemetry.addLine("Panels controls the PIDF values.");
        telemetry.addLine("Set RUN_FLYWHEEL = true to spin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driver.update();

            flywheel.setVelocity(TARGET_VELOCITY);
            flywheel.update();

            if(driver.isDown(RumbleGamepad.Trigger.RIGHT_TRIGGER)){
                gate.open();
            }else{
                gate.close();
            }

            double currentVelocity = flywheel.getVelocity();

            double error = TARGET_VELOCITY - currentVelocity;

            intakeTransfer.setPower(1);

            panelsTelemetry.debug(
                    "current: " + currentVelocity,
                    "target: " + TARGET_VELOCITY,
                    "error: " + error
            );

            //telemetry
            telemetry.addData("Target Velocity", "%.2f ticks/s", TARGET_VELOCITY);
            telemetry.addData("Current Velocity", "%.2f ticks/s", currentVelocity);
            telemetry.addData("Error", "%.2f ticks/s", error);
            telemetry.addData("Power", "%.3f", flywheel.getPower());
            telemetry.addLine("");
            telemetry.addData("kP", flywheel.CURRENT_FLYWHEEL_KP);
            telemetry.addData("kI", FLYWHEEL_KI);
            telemetry.addData("kD", FLYWHEEL_KD);
            telemetry.addData("kF", FLYWHEEL_KF);
            telemetry.addLine("");
            telemetry.addData("Running", RUN_FLYWHEEL);

            panelsTelemetry.update(telemetry);
        }

        flywheel.stop();
    }
}
