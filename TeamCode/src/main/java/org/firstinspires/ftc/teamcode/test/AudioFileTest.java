
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.File;

@TeleOp(name = "Audio File Test", group = "Test")
public class AudioFileTest extends LinearOpMode {

    private final String SOUND_PATH = "/sdcard/FIRST/blocks/sounds/";

    private File shooterReady =
            new File(SOUND_PATH + "ding.mp3");

    @Override
    public void runOpMode() {

        telemetry.addData("Shooter Ready",
                shooterReady.exists() ? "Found" : "Missing");

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.aWasPressed()) {
                if (shooterReady.exists()) {
                    SoundPlayer.getInstance().startPlaying(
                            hardwareMap.appContext,
                            shooterReady
                    );
                }
            }
        }
    }
}
