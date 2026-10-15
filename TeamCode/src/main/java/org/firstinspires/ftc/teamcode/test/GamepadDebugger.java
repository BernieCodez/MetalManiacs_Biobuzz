package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.RumbleGamepad;
@TeleOp(name = "Gamepad Debugger", group = "Z - Test")
public class GamepadDebugger extends LinearOpMode {
    RumbleGamepad gp;
    @Override
    public void runOpMode() {
        gp = new RumbleGamepad(gamepad1);

        waitForStart();

        if (isStopRequested()) return;

        //Run the selected option
        while (opModeIsActive()) {
            gp.update();
            telemetry.addData("Left Stick X", gp.leftX());
            telemetry.addData("Left Stick Y", gp.leftY());
            telemetry.addData("Right Stick X", gp.rightX());
            telemetry.addData("Right Stick Y", gp.rightY());
            telemetry.addData("A", gp.wasJustPressed(RumbleGamepad.Button.A));
            telemetry.addData("B", gp.wasJustPressed(RumbleGamepad.Button.B));
            telemetry.addData("X", gp.wasJustPressed(RumbleGamepad.Button.X));
            telemetry.addData("Y", gp.wasJustPressed(RumbleGamepad.Button.Y));
            telemetry.update();
        }
    }
}
