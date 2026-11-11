package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Gate;
import org.firstinspires.ftc.teamcode.hardware.IntakeTransfer;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//@Disabled
@TeleOp(name = "Flywheel PID/Velocity", group = "A - Test")
public class RunFlywheelsPID extends LinearOpMode {
    public static double MIN_VELOCITY = 700;
    public static double MAX_VELOCITY = 2500;
    public static double VELOCITY_STEP = 50;
    public Flywheel flywheel;
    public IntakeTransfer intakeTransfer;
    public Gate gate;
    RumbleGamepad driver;
    public double tps;
    @Override
    public void runOpMode() {
        flywheel = new Flywheel(hardwareMap);
        intakeTransfer = new IntakeTransfer(hardwareMap);
        gate = new Gate(hardwareMap);
        driver = new RumbleGamepad(gamepad1);
        tps = 1000;

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driver.update();
            if (driver.wasJustPressed(RumbleGamepad.Button.RIGHT_BUMPER)){
                tps += VELOCITY_STEP;
            } else if (driver.wasJustPressed(RumbleGamepad.Button.LEFT_BUMPER)) {
                tps-=VELOCITY_STEP;
            }

            if(driver.wasJustPressed(RumbleGamepad.Trigger.RIGHT_TRIGGER)){
                gate.open();
            }
            if(driver.wasJustPressed(RumbleGamepad.Trigger.LEFT_TRIGGER)){
                gate.close();
            }
            tps = Math.max(MIN_VELOCITY, Math.min(tps, MAX_VELOCITY));

            flywheel.setVelocity(tps);
            intakeTransfer.setPower(1);

            telemetry.addLine("Use bumpers to control motor velocity");
            telemetry.addData("Target velocity", tps);
            telemetry.addData("Left velocity", flywheel.getLeftVelocity());
            telemetry.addData("Right velocity", flywheel.getRightVelocity());
            telemetry.addData("Average velocity", flywheel.getVelocity());
            telemetry.addData("Target velocity", flywheel.getTargetVelocity());
            telemetry.update();
        }

        flywheel.stop();
    }
}
