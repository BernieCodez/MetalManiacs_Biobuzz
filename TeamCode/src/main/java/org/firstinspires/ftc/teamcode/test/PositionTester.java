package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad.Button;

@TeleOp(name = "Position Tester", group = "TeleOp")
public class PositionTester extends OpMode {

    private static final double STICK_DEADZONE = 0.1;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose[] startPoses = {
            poseFactory.of(0, 0, 0),
            poseFactory.of(141.5, 141.5, 0)
    };

    private final String[] targetNames = {"shootOne", "pickUpPollen2", "shootTwo", "parkAtEnd"};
    private final Pose[] targets = {
            poseFactory.of(87, 139, 90),
            poseFactory.of(136.1937, 139.1417, 90.0195),
            poseFactory.of(84.435, 31.0886, -95.6534),
            poseFactory.of(133.6, 26, -86.3588)
    };
    private final Button[] targetButtons = {Button.CROSS, Button.CIRCLE, Button.SQUARE, Button.TRIANGLE};

    private Follower follower;
    private RumbleGamepad driver;

    private int startIndex = 0;
    private double startHeadingDeg = 0;
    private Pose savedPose = null;
    private String activeTarget = "none";
    private boolean driving = false;
    private double speedMultiplier = 1;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        driver = new RumbleGamepad(gamepad1);
    }

    @Override
    public void init_loop() {
        driver.update();

        if (driver.wasJustPressed(Button.DPAD_LEFT)) startIndex = 0;
        if (driver.wasJustPressed(Button.DPAD_RIGHT)) startIndex = 1;
        if (driver.wasJustPressed(Button.DPAD_UP)) startHeadingDeg = wrapDegrees(startHeadingDeg + 90);
        if (driver.wasJustPressed(Button.DPAD_DOWN)) startHeadingDeg = wrapDegrees(startHeadingDeg - 90);

        Pose start = selectedStart();
        follower.setPose(start);
        follower.update();

        telemetry.addLine("DPAD LEFT = (0, 0)   DPAD RIGHT = (141.5, 141.5)");
        telemetry.addLine("DPAD UP / DOWN = rotate start heading 90");
        telemetry.addData("Start", "(%.1f, %.1f) @ %.0f deg", start.x(), start.y(), startHeadingDeg);
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(selectedStart());
        follower.update();
    }

    @Override
    public void loop() {
        driver.update();
        follower.update();
        Pose robot = follower.pose();

        if (driver.wasJustPressed(Button.RIGHT_BUMPER)) {
            savedPose = robot;
        }

        for (int i = 0; i < targets.length; i++) {
            if (driver.wasJustPressed(targetButtons[i])) {
                driveTo(robot, targets[i], targetNames[i]);
            }
        }
        if (driver.wasJustPressed(Button.LEFT_BUMPER) && savedPose != null) {
            driveTo(robot, savedPose, "saved");
        }

        if (driver.wasJustPressed(Button.OPTION)) {
            speedMultiplier = speedMultiplier == 1 ? 0.5 : 1;
        }

        boolean sticksMoved = Math.abs(driver.leftX()) > STICK_DEADZONE
                || Math.abs(driver.leftY()) > STICK_DEADZONE
                || Math.abs(driver.rightX()) > STICK_DEADZONE;

        if (driving && (sticksMoved || driver.wasJustPressed(Button.DPAD_DOWN))) {
            follower.hold(robot);
            driving = false;
            activeTarget = "cancelled";
        }

        if (driving) {
            if (!follower.following()) {
                driving = false;
            }
        } else {
            double exponent = 3;
            double throttle = RumbleGamepad.exponentialInput(driver.leftY(), exponent) * speedMultiplier;
            double strafe = RumbleGamepad.exponentialInput(-driver.leftX(), exponent) * speedMultiplier;
            double rotation = RumbleGamepad.exponentialInput(-driver.rightX(), exponent) * speedMultiplier;
            ManualDrive.driveOrHold(follower, new DrivePowers(throttle, strafe, rotation));
        }

        telemetry.addData("x", robot.x());
        telemetry.addData("y", robot.y());
        telemetry.addData("heading", robot.heading());
        telemetry.addData("heading (deg)", Math.toDegrees(robot.heading()));
        telemetry.addData("pose", "poseFactory.of(%.4f, %.4f, %.4f)", robot.x(), robot.y(), Math.toDegrees(robot.heading()));
        telemetry.addLine("");
        telemetry.addData("Start", "(%.1f, %.1f) @ %.0f deg", startPoses[startIndex].x(), startPoses[startIndex].y(), startHeadingDeg);
        telemetry.addData("Target", activeTarget);
        telemetry.addData("Driving to target", driving);
        if (driving && follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
        }
        if (savedPose != null) {
            telemetry.addData("Saved", "(%.2f, %.2f) @ %.1f deg", savedPose.x(), savedPose.y(), Math.toDegrees(savedPose.heading()));
        }
        telemetry.addData("Speed", speedMultiplier == 1 ? "Fast" : "Slow");
        telemetry.addLine("");
        telemetry.addLine("X shootOne  O pickUpPollen2  [] shootTwo  /\\ parkAtEnd");
        telemetry.addLine("RB save pose  LB go to saved  OPTIONS speed");
        telemetry.addLine("Move a stick or DPAD DOWN to cancel");
        telemetry.update();
    }

    private void driveTo(Pose robot, Pose target, String name) {
        Path path = Paths.line(robot, target).constant(target.heading());
        follower.follow(path);
        driving = true;
        activeTarget = name;
    }

    private Pose selectedStart() {
        Pose base = startPoses[startIndex];
        return poseFactory.of(base.x(), base.y(), startHeadingDeg);
    }

    private static double wrapDegrees(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }
}
