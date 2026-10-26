package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ConfigKeybinds extends LinearOpMode {

    private String getButtonPress() {
        if (gamepad1.a) {
            return "a";
        }
        if (gamepad1.b) {
            return "b";
        }
        if (gamepad1.x) {
            return "x";
        }
        if (gamepad1.y) {
            return "y";
        }

        if (gamepad1.dpad_up) {
            return "dpad_up";
        }
        if (gamepad1.dpad_down) {
            return "dpad_down";
        }
        if (gamepad1.dpad_left) {
            return "dpad_left";
        }
        if (gamepad1.dpad_right) {
            return "dpad_right";
        }
        if (gamepad1.left_bumper) {
            return "left_bumper";
        }
        if (gamepad1.right_bumper) {
            return "right_bumper";
        }
        if (gamepad1.start) {
            return "start";
        }
        if (gamepad1.options) {
            return "options";
        }
        if (gamepad1.left_stick_button) {
            return "left_stick_button";
        }
        if (gamepad1.right_stick_button) {
            return "right_stick_button";
        }
        if (gamepad1.right_trigger > 0.2) {
            return "right_trigger";
        }
        if (gamepad1.right_trigger < -0.2) {
            return "right_trigger";
        }
        if (gamepad1.left_trigger > 0.2) {
            return "left_trigger";
        }
        if (gamepad1.left_trigger < -0.2) {
            return "left_trigger";
        }
        return "";
    }

    // Helper method to display a styled prompt on the Driver Station
    private void displayPrompt(String actionName) {
        telemetry.clearAll();
        telemetry.addLine("<font color='#00FFFF'><b>=== KEYBIND CONFIGURATION ===</b></font>");
        telemetry.addLine("Press a button or trigger for:");
        telemetry.addData(">> TARGET ACTION", "<b><font color='yellow'>" + actionName.toUpperCase() + "</font></b>");
        telemetry.addLine("---------------------------------------");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        String intakeButton = "";
        String outakeButton = "";
        String fire = "";
        String fieldCentric = "";
        String reverseIntake = "";
        String autoAim = "";

        telemetry.addLine("<font color='yellow'><b>[CONFIG MODE READY]</b> Press Init/Play to start</font>");
        telemetry.update();

        waitForStart();

        // 1. Intake Mapping
        displayPrompt("Intake");
        while (opModeIsActive() && intakeButton.equals("")) {
            String pressedButton = getButtonPress();
            if (!pressedButton.equals("")) {
                intakeButton = pressedButton;
                while (!getButtonPress().equals("")) {
                    idle();
                }
            }
        }

        // 2. Outtake Mapping
        displayPrompt("Outtake");
        while (opModeIsActive() && outakeButton.equals("")) {
            String pressedButton = getButtonPress();
            if (!pressedButton.equals("")) {
                outakeButton = pressedButton;
                while (!getButtonPress().equals("")) {
                    idle();
                }
            }
        }

        // 3. Fire Mapping
        displayPrompt("Fire");
        while (opModeIsActive() && fire.equals("")) {
            String pressedButton = getButtonPress();
            if (!pressedButton.equals("")) {
                fire = pressedButton;
                while (!getButtonPress().equals("")) {
                    idle();
                }
            }
        }

        // 4. Field Centric Mapping
        displayPrompt("Field Centric");
        while (opModeIsActive() && fieldCentric.equals("")) {
            String pressedButton = getButtonPress();
            if (!pressedButton.equals("")) {
                fieldCentric = pressedButton;
                while (!getButtonPress().equals("")) {
                    idle();
                }
            }
        }

        // 5. Reverse Intake Mapping
        displayPrompt("Reverse Intake");
        while (opModeIsActive() && reverseIntake.equals("")) {
            String pressedButton = getButtonPress();
            if (!pressedButton.equals("")) {
                reverseIntake = pressedButton;
                while (!getButtonPress().equals("")) {
                    idle();
                }
            }
        }

        // 6. Auto Aim Mapping
        displayPrompt("Auto Aim");
        while (opModeIsActive() && autoAim.equals("")) {
            String pressedButton = getButtonPress();
            if (!pressedButton.equals("")) {
                autoAim = pressedButton;
                while (!getButtonPress().equals("")) {
                    idle();
                }
            }
        }

        // Styled Final Summary Screen
        telemetry.clearAll();
        telemetry.addLine("<font color='#00FF00'><b>=== CONFIGURATION COMPLETE ===</b></font>");
        telemetry.addLine("---------------------------------------");
        telemetry.addData("  • Intake", intakeButton.toUpperCase());
        telemetry.addData("  • Outtake", outakeButton.toUpperCase());
        telemetry.addData("  • Fire", fire.toUpperCase());
        telemetry.addData("  • Field Centric", fieldCentric.toUpperCase());
        telemetry.addData("  • Reverse Intake", reverseIntake.toUpperCase());
        telemetry.addData("  • Auto Aim", autoAim.toUpperCase());
        telemetry.addLine("---------------------------------------");
        telemetry.addLine("<font color='gray'>Press STOP on Driver Station when ready.</font>");
        telemetry.update();

        // Hold loop to keep summary up
        while (opModeIsActive()) {
            idle();
        }
    }
}