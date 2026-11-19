package org.firstinspires.ftc.teamcode.util;

import android.content.Context;
import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.Config.Alliance;
import org.firstinspires.ftc.teamcode.Profile;

import java.util.ArrayList;

@TeleOp(name = "Prematch Setup", group = "C - Util")
public class PrematchSetup extends LinearOpMode {

    private static SharedPreferences savedAnswers() {
        return AppUtil.getDefContext().getSharedPreferences("FTC_PREFS", Context.MODE_PRIVATE);
    }

    public static int loadInt(String key, int fallback) {
        try {
            return savedAnswers().getInt(key, fallback);
        } catch (RuntimeException e) {
            return fallback;
        }
    }

    public static boolean loadBoolean(String key, boolean fallback) {
        try {
            return savedAnswers().getBoolean(key, fallback);
        } catch (RuntimeException e) {
            return fallback;
        }
    }

    public static double loadDouble(String key, double fallback) {
        try {
            return savedAnswers().getFloat(key, (float) fallback);
        } catch (RuntimeException e) {
            return fallback;
        }
    }

    public static String loadText(String key, String fallback) {
        try {
            return savedAnswers().getString(key, fallback);
        } catch (RuntimeException e) {
            return fallback;
        }
    }

    public static Alliance loadAlliance() {
        if (loadText("alliance", "BLUE").equals("RED")) return Alliance.RED;
        return Alliance.BLUE;
    }

    public static Profile loadDriver(Profile[] drivers) {
        String name = loadText("driver", "");
        for (Profile driver : drivers) {
            if (driver.name.equals(name)) return driver;
        }
        return drivers[0];
    }

    private static boolean saveAnswers() {
        try {
            SharedPreferences.Editor editor = savedAnswers().edit();
            editor.putString("driver", Drivers.activeDriver.name);
            editor.putString("alliance", Config.currentAlliance.name());
            return editor.commit();
        } catch (RuntimeException e) {
            return false;
        }
    }

    private static int wrap(int value, int count) {
        if (value < 0) return count - 1;
        if (value >= count) return 0;
        return value;
    }

    private static ArrayList<String> questions() {
        ArrayList<String> rows = new ArrayList<>();
        rows.add("Driver");
        rows.add("Alliance");
        return rows;
    }

    private static String answer(String question) {
        if (question.equals("Driver")) return Drivers.activeDriver.name;
        if (question.equals("Alliance")) return Config.currentAlliance.name();
        return "";
    }

    private static void changeAnswer(String question, int direction) {
        if (question.equals("Driver")) {
            int index = 0;
            for (int i = 0; i < Drivers.DRIVERS.length; i++) {
                if (Drivers.DRIVERS[i] == Drivers.activeDriver) index = i;
            }
            index = wrap(index + direction, Drivers.DRIVERS.length);
            Drivers.activeDriver = Drivers.DRIVERS[index];
        }
        if (question.equals("Alliance")) {
            if (Config.currentAlliance == Alliance.BLUE) {
                Config.currentAlliance = Alliance.RED;
            } else {
                Config.currentAlliance = Alliance.BLUE;
            }
        }
    }

    @Override
    public void runOpMode() {
        RumbleGamepad gp = new RumbleGamepad(gamepad1);
        int selected = 0;
        boolean saveFailed = false;

        while (opModeInInit()) {
            gp.update();
            ArrayList<String> rows = questions();

            if (gp.wasJustPressed(RumbleGamepad.Button.DPAD_UP)) selected = selected - 1;
            if (gp.wasJustPressed(RumbleGamepad.Button.DPAD_DOWN)) selected = selected + 1;
            if (selected < 0) selected = 0;
            if (selected > rows.size() - 1) selected = rows.size() - 1;

            int direction = 0;
            if (gp.wasJustPressed(RumbleGamepad.Button.DPAD_RIGHT)) direction = 1;
            if (gp.wasJustPressed(RumbleGamepad.Button.DPAD_LEFT)) direction = -1;
            if (direction != 0) {
                changeAnswer(rows.get(selected), direction);
                saveFailed = !saveAnswers();
            }

            telemetry.addLine("=== PREMATCH SETUP ===");
            if (saveFailed) telemetry.addLine("WARNING: could not save, answers reset on restart");
            for (int i = 0; i < rows.size(); i++) {
                String arrow = "   ";
                if (i == selected) arrow = "> ";
                telemetry.addLine(arrow + rows.get(i) + ":  " + answer(rows.get(i)));
            }
            telemetry.addLine("");
            telemetry.addLine("D-pad up/down: question   left/right: answer");
            telemetry.addLine("Answers save automatically");
            telemetry.update();

            idle();
        }
    }
}
