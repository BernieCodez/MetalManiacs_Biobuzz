
package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.hardware.Limelight;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static org.firstinspires.ftc.teamcode.Config.*;

@TeleOp(name = "Limelight Correction Test", group = "A - Test")
public class LimelightTest extends OpMode {

    private Follower follower;
    private Limelight limelight;

    private boolean correctionEnabled = true;
    private boolean inverted = true;

    private boolean previousA = false;
    private boolean previousB = false;

    // Rotation values
    private double commandedRotation = 0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        limelight = new Limelight(hardwareMap);

        telemetry.addLine("Limelight Correction Test");
        telemetry.addLine("A: Toggle correction");
        telemetry.addLine("B: Reverse correction direction");
        telemetry.addLine("Robot rotation is controlled ONLY by Limelight TX.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        Pose robot = follower.pose();

        Alliance.Hive activeHive = robot.y() > HIVE_BOUNDARY ? currentAlliance.TOP : currentAlliance.BOTTOM;

        limelight.update(activeHive);

        //toggle correction with A
        if (gamepad1.a && !previousA) {
            correctionEnabled = !correctionEnabled;
        }
        previousA = gamepad1.a;

        //reverse correction direction with B
        if (gamepad1.b && !previousB) {
            inverted = !inverted;
        }
        previousB = gamepad1.b;

        commandedRotation = 0;

        if (correctionEnabled && limelight.tVisible) {
            double direction = inverted ? -1.0 : 1.0;

            commandedRotation = clip(
                    AUTOAIM_P * limelight.tx * direction,
                    -AUTOAIM_MAX_ROTATION_SPEED,
                    AUTOAIM_MAX_ROTATION_SPEED
            );
        }

        DrivePowers powers = new DrivePowers(
                0,
                0,
                commandedRotation
        );
        ManualDrive.driveOrHold(follower, powers);

        telemetry.addLine("=== LIMELIGHT ===");
        telemetry.addData("Target Visible", limelight.tVisible);
        telemetry.addData("TX (degrees)", "%.2f", limelight.tx);
        telemetry.addData("TY (degrees)", "%.2f", limelight.ty);

        telemetry.addLine("");
        telemetry.addLine("=== TEST CONTROLLER ===");
        telemetry.addData("Correction Enabled", correctionEnabled);
        telemetry.addData("Direction Inverted", inverted);
        telemetry.addData("Commanded Rotation", "%.4f", commandedRotation);

        telemetry.addLine("");
        telemetry.addLine("=== LOCALIZATION (DISPLAY ONLY) ===");
        telemetry.addData("Robot X", "%.2f", robot.x());
        telemetry.addData("Robot Y", "%.2f", robot.y());
        telemetry.addData("Robot Heading", "%.1f degrees", Math.toDegrees(robot.heading()));
        telemetry.addData("Active Hive", activeHive.name);

        telemetry.addLine("");
        telemetry.addLine("A = Toggle correction | B = Reverse direction");

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }
}
