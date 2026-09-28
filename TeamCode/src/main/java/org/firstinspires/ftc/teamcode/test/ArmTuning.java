package org.firstinspires.ftc.teamcode.test;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name = "Arm Tuning", group = "Tuning")
public class ArmTuning extends LinearOpMode {

    // PID
    public static double P = 0.0;
    public static double I = 0.0;
    public static double D = 0.0;

    // Gravity feedforward
    public static double F = 0.0;

    // Target arm position in encoder ticks
    public static int targetPosition = 0;
    public static double TICKS_PER_REV = 600;

    private DcMotorEx armMotor;


    @Override
    public void runOpMode() throws InterruptedException {

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        armMotor.setDirection(DcMotorEx.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Init complete");
        telemetry.update();

        waitForStart();

        double integral = 0;
        double previousError = 0;

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {

            double dt = timer.seconds();
            timer.reset();

            // Current position
            double currentPosition = armMotor.getCurrentPosition();

            // Position error
            double error = targetPosition - currentPosition;

            // Integral
            integral += error * dt;

            // Derivative
            double derivative = 0;

            if (dt > 0) {
                derivative = (error - previousError) / dt;
            }

            previousError = error;

            // Convert encoder position to angle
            // You need to change TICKS_PER_REV to your motor/gearbox setup.
            double angle = (currentPosition / TICKS_PER_REV) * 2 * Math.PI;

            // Gravity feedforward
            double feedforward = F * Math.cos(angle);

            // PID + feedforward
            double output =
                    P * error
                            + I * integral
                            + D * derivative
                            + feedforward;

            // Limit motor power
            output = Math.max(-1, Math.min(1, output));

            armMotor.setPower(output);


            // Driver Station
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Position", currentPosition);
            telemetry.addData("Error", error);
            telemetry.addData("Output", output);
            telemetry.addData("Feedforward", feedforward);

            telemetry.addData("P", P);
            telemetry.addData("I", I);
            telemetry.addData("D", D);
            telemetry.addData("F", F);

            telemetry.update();
        }
    }
}