package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PushbotArm {
        //PIDF
        public static double P = 0.003;//proportinal
        public static double I = 0;//integral
        public static double D = 0.0;//derivative
        public static double F = 0.4;//feedforward

        //target arm position in encoder ticks
        public int targetPosition = 0;
        public static double TICKS_PER_REV = 600;

        private DcMotorEx armMotor;
        private double currentPosition;
        public PushbotArm(HardwareMap hardwareMap) {
            armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
            armMotor.setDirection(DcMotorEx.Direction.FORWARD);

            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            currentPosition = armMotor.getCurrentPosition();
        }
        public double getArmPosition() {
            return armMotor.getCurrentPosition();
        }
        public void update() {

            double integral = 0;
            double previousError = 0;

            ElapsedTime timer = new ElapsedTime();


                double dt = timer.seconds();
                timer.reset();

                //current position
                currentPosition = armMotor.getCurrentPosition();

                //position error
                double error = targetPosition - currentPosition;

                //integral
                integral += error * dt;

                //derivative
                double derivative = 0;

                if (dt > 0) {
                    derivative = (error - previousError) / dt;
                }

                previousError = error;

                //convert encoder position to angle
                double angle = (currentPosition / TICKS_PER_REV) * 2 * Math.PI;

                //gravity feedforward
                double feedforward = F * Math.cos(angle);

                //PID + feedforward
                double output =
                        P * error
                                + I * integral
                                + D * derivative
                                + feedforward;

                //limit motor power
                output = Math.max(-1, Math.min(1, output));

                armMotor.setPower(output);
        }
}
