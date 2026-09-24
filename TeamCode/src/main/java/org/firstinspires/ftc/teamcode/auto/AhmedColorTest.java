package org.firstinspires.ftc.teamcode.auto;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "AhmedColorTest", group = "Autonomous")
public class AhmedColorTest extends LinearOpMode {

    NormalizedColorSensor colorSensor;
    View relativeLayout;

    @Override
    public void runOpMode() {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        try {
            runSample();
        } finally {
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });
        }
    }

    protected void runSample() {
        float gain = 2;
        final float[] hsvValues = new float[3];
        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed = false;

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
            telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

            if (gamepad1.a) {
                gain += 0.005;
            } else if (gamepad1.b && gain > 1) {
                gain -= 0.005;
            }
            telemetry.addData("Gain", gain);
            colorSensor.setGain(gain);

            xButtonCurrentlyPressed = gamepad1.x;
            if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
                if (xButtonCurrentlyPressed) {
                    if (colorSensor instanceof SwitchableLight) {
                        SwitchableLight light = (SwitchableLight)colorSensor;
                        light.enableLight(!light.isLightOn());
                    }
                }
            }
            xButtonPreviouslyPressed = xButtonCurrentlyPressed;

            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            float hueRange = 20;

            if (true){//hsvValues[1] > 0.20) {
                if (inRange(hsvValues[0], 100, hueRange)) {
                    telemetry.addLine("yellow");
                } else {
                    telemetry.addLine("No wanted colors found");
                }
            } else {
                telemetry.addLine("Saturation too low");
            }

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsvValues[0])
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }

            telemetry.update();

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
                }
            });
        }
    }

    public boolean inRange (float inputHue, float expectedHue, float range){
        if (inputHue+range>=360){
            inputHue=(360-inputHue)*-1;
        }
        if (inputHue>=expectedHue-range && inputHue<=expectedHue+range){
            return true;
        } else{
            return false;
        }
    }
}