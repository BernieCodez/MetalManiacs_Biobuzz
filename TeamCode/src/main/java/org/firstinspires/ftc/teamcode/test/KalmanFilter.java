package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Kalman Filter")
public class KalmanFilter extends LinearOpMode {

    private DistanceSensor distanceSensor;

    //variables
    double estimate = 0;
    double estimateError = 1;

    // Lower = trust our prediction more
    double processNoise = 0.01;

    // Higher = trust the sensor less
    double measurementNoise = 2.0;

    @Override
    public void runOpMode() {

        distanceSensor = hardwareMap.get(
                DistanceSensor.class,
                "distanceSensor"
        );

        waitForStart();

        while (opModeIsActive()) {

            // Get reading
            double rawDistance =
                    distanceSensor.getDistance(DistanceUnit.CM);

            // KALMAN FILTER

            // 1. Prediction
            estimateError += processNoise;

            // 2. Calculate Kalman Gain
            double kalmanGain =
                    estimateError /
                            (estimateError + measurementNoise);

            // 3. Correct the estimate
            estimate +=
                    kalmanGain *
                            (rawDistance - estimate);

            // 4. Update uncertainty
            estimateError =
                    (1 - kalmanGain) * estimateError;

            // TELEMETRY
            telemetry.addData("Raw Distance", rawDistance);
            telemetry.addData("Filtered Distance", estimate);
            telemetry.addData("Kalman Gain", kalmanGain);
            telemetry.addData("Error", estimateError);

            telemetry.update();
        }
    }
}