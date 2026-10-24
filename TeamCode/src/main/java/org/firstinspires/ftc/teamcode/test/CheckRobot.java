package org.firstinspires.ftc.teamcode.test;

import com.pedropathing.revhub.localizers.PinpointLocalizer;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
//©JavaDude
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.deprecated.ColorSensor;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import static org.firstinspires.ftc.teamcode.Config.*;

@TeleOp(name = "Check Robot", group = "Z - Test")
public class CheckRobot extends LinearOpMode {

    // SETTINGS - change these to adjust the tests
    // how hard and how long each motor and servo runs
    private static final double DRIVE_POWER = 0.3;
    private static final double DRIVE_SPIN_SECONDS = 1.5;
    private static final double PUSH_INCHES = 24;
    private static final double TURN_DEGREES = 90;
    private static final double INTAKE_POWER = -0.5;
    private static final double INTAKE_SPIN_SECONDS = 1.0;
    private static final double FLYWHEEL_POWER = -0.5;
    private static final double FLYWHEEL_SPIN_SECONDS = 1.5;
    private static final double LEFT_FLYWHEEL_ENCODER_SIGN = 1;
    private static final double RIGHT_FLYWHEEL_ENCODER_SIGN = -1;
    private static final double SERVO_MOVE_SECONDS = 0.7;

    // pass/fail limits
    private static final double LOW_BATTERY_VOLTS = 11.4;
    private static final double MIN_MOTOR_AMPS = 0.1;
    private static final double MIN_FLYWHEEL_TICKS_PER_SECOND = 200;
    private static final double POD_MIN_MOVE_INCHES = 2;
    private static final double TURN_MIN_DEGREES = 10;

    // what the results say on the screen
    private static final String PASS = "✅ PASS";
    private static final String FAIL = "❌ FAIL";
    private static final String SKIPPED = "➖ SKIPPED";
    private static final String WAITING = "⏳ waiting";

    // LIST OF TESTS - remove a name to remove a test, they run in this order
    private final String[] tests = {"Battery", "Drivetrain", "Pinpoint", "Intake", "Flywheels", "Gate", "Hood",
            "Wedges", "Color sensor", "Limelight"};

    // what each test in the list runs (the names must match the list above)
    private String runTest(String test) {
        switch (test) {
            case "Battery":          return testBattery();
            case "Drivetrain":       return testDrivetrain();
            case "Pinpoint":         return testPinpoint();
            case "Intake":           return testIntake();
            case "Flywheels":        return testFlywheels();
            case "Gate":             return testServos("gate", GATE_OPEN, GATE_CLOSE, GATE);
            case "Hood":             return testServos("hood", HOOD_NECTAR, HOOD_POLLEN, HOOD);
            case "Wedges":           return testServos("wedges", WEDGE_DOWN, WEDGE_UP, LEFT_WEDGE, RIGHT_WEDGE);
            case "Color sensor":     return testColorSensor();
            case "Limelight":        return testLimelight();
        }
        return FAIL + " - \"" + test + "\" is in the list but has no test in runTest()";
    }

    // the code below runs the tests, you usually don't need to change it
    private final boolean[] selected = new boolean[tests.length]; // true = checked in the menu
    private final String[] results = new String[tests.length];
    private String title = ""; // the line at the top of the screen

    private ColorSensor colorSensor;
    private Limelight3A limelight;
    // these turn true the first time the sensor sees something, so the test passes even if you let go before pressing A
    private boolean colorSensorSawBall = false;
    private boolean limelightSawTag = false;

    private GoBildaPinpointDriver pinpoint;
    private double forwardInches, leftInches, turnedDegrees;

    // SCREEN FLOW - pick tests, run them, show the summary
    @Override
    public void runOpMode() {
        setAll(true); // everything starts checked
        chooseTests(); // menu shows until START is pressed
        startLiveSensors();
        runTests(false);

        // after the tests finish, keep showing the results until STOP is pressed
        while (opModeIsActive()) {
            showSummary();
        }

        if (limelight != null) limelight.stop();
    }

    private void chooseTests() {
        int cursor = 0; // which line the 👉 is pointing at
        while (opModeInInit()) {
            // "WasPressed" is only true once per press, so holding a button doesn't keep toggling
            if (gamepad1.dpadUpWasPressed() && cursor > 0) cursor--;
            if (gamepad1.dpadDownWasPressed() && cursor < tests.length - 1) cursor++;
            if (gamepad1.aWasPressed()) selected[cursor] = !selected[cursor];
            if (gamepad1.xWasPressed()) setAll(false);
            if (gamepad1.yWasPressed()) setAll(true);

            telemetry.addLine("ROBOT CHECK - pick what to test");
            telemetry.addLine("DPAD up/down = move,  A = check/uncheck");
            telemetry.addLine("X = uncheck all,  Y = check all");
            telemetry.addLine("Then press START (play button)");
            telemetry.addLine();
            for (int i = 0; i < tests.length; i++) {
                String line = "      ";
                if (i == cursor) line = "👉 ";
                if (selected[i]) line += "☑ ";
                else line += "☐ ";
                telemetry.addLine(line + tests[i]);
            }
            telemetry.update();
        }
    }

    // onlyFailed = true when re-testing from the summary screen, so passed tests aren't run again
    private void runTests(boolean onlyFailed) {
        int total = 0;
        for (int i = 0; i < tests.length; i++) {
            if (!selected[i]) continue;
            if (onlyFailed && !results[i].startsWith(FAIL)) continue;
            results[i] = WAITING;
            total++;
        }

        int number = 0;
        for (int i = 0; i < tests.length && opModeIsActive(); i++) {
            if (!WAITING.equals(results[i])) continue;
            number++;
            title = "TEST " + number + " OF " + total + ": " + tests[i].toUpperCase();
            results[i] = "🔄 testing now";
            results[i] = runTest(tests[i]); // runTest returns the PASS / FAIL / SKIPPED text
        }
    }

    private void showSummary() {
        int passed = 0;
        int failed = 0;
        for (String result : results) {
            if (result == null) continue;
            if (result.startsWith(PASS)) passed++;
            if (result.startsWith(FAIL)) failed++;
        }

        title = "ALL DONE: " + passed + " passed, " + failed + " failed";
        if (failed == 0) {
            show("Everything works! Press STOP");
            return;
        }

        show("Fix the problems below, then press A to test them again");
        if (gamepad1.aWasPressed()) runTests(true);
    }

    // TESTS - one method per test, in the order they run
    private String testBattery() {
        // the hub measures the battery voltage, this grabs that reading
        double volts = hardwareMap.voltageSensor.iterator().next().getVoltage();
        if (volts <= LOW_BATTERY_VOLTS) {
            return FAIL + " - battery low (" + round(volts) + "V). Swap in a charged battery";
        }
        return PASS + " (" + round(volts) + "V)";
    }

    private String testIntake() {
        if (!pressAorB("Keep hands away from the intake", "A = start,  B = skip")) {
            return SKIPPED;
        }
        return testMotors(INTAKE_POWER, INTAKE_SPIN_SECONDS, null, INTAKE_TRANSFER);
    }

    private String testFlywheels() {
        if (!pressAorB("Keep hands away from the flywheels", "A = start,  B = skip")) {
            return SKIPPED;
        }
        double[] encoderSigns = {LEFT_FLYWHEEL_ENCODER_SIGN, RIGHT_FLYWHEEL_ENCODER_SIGN};
        return testMotors(FLYWHEEL_POWER, FLYWHEEL_SPIN_SECONDS, encoderSigns, LEFT_FLYWHEEL, RIGHT_FLYWHEEL);
    }

    // spins motors and checks each one gets power (and for flywheels, spins the right way)
    private String testMotors(double power, double seconds, double[] encoderSigns, String... motorNames) {
        boolean checkSpinSpeed = encoderSigns != null;
        DcMotorEx[] motors = new DcMotorEx[motorNames.length];
        double[] highestAmps = new double[motorNames.length];

        // find every motor and start them all spinning at the same time
        for (int i = 0; i < motorNames.length; i++) {
            // tryGet gives null instead of crashing when the name isn't in the robot config
            motors[i] = hardwareMap.tryGet(DcMotorEx.class, motorNames[i]);
            if (motors[i] == null) return notInConfig(motorNames[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // plain power, no speed control
            motors[i].setPower(power);
        }

        // while they spin, remember the most current (amps) each motor pulled
        // a motor that's unplugged pulls no current at all
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            for (int i = 0; i < motorNames.length; i++) {
                highestAmps[i] = Math.max(highestAmps[i], motors[i].getCurrent(CurrentUnit.AMPS));
            }
            show("Spinning...");
        }

        // stop each motor and write down anything that went wrong
        String problems = "";
        for (int i = 0; i < motorNames.length; i++) {
            double speed = motors[i].getVelocity() * Math.signum(power); // read the speed BEFORE stopping it
            if (checkSpinSpeed) speed *= encoderSigns[i];
            motors[i].setPower(0);
            // getConnectionInfo says which hub and port it's plugged into, e.g. "Control Hub port 2"
            String motor = motorNames[i] + " (" + motors[i].getConnectionInfo() + ")";

            if (highestAmps[i] < MIN_MOTOR_AMPS) {
                problems += motor + " got no power. Check its power wire is plugged in. ";
            } else if (checkSpinSpeed && speed < -MIN_FLYWHEEL_TICKS_PER_SECOND) {
                problems += motor + " spins backwards. Swap its power wires. ";
            } else if (checkSpinSpeed && speed < MIN_FLYWHEEL_TICKS_PER_SECOND) {
                problems += motor + " not spinning. Check its encoder wire and that nothing is jammed. ";
            }
        }

        if (problems.isEmpty()) return PASS;
        return FAIL + " - " + problems;
    }

    // moves servos back and forth, then asks if they moved
    private String testServos(String name, double position1, double position2, String... servoNames) {
        Servo[] servos = new Servo[servoNames.length];
        for (int i = 0; i < servoNames.length; i++) {
            servos[i] = hardwareMap.tryGet(Servo.class, servoNames[i]);
            if (servos[i] == null) return notInConfig(servoNames[i]);
        }

        if (!pressAorB("Watch the " + name + ", it will move back and forth", "A = start,  B = skip")) {
            return SKIPPED;
        }

        // servos can't tell us where they are, so we move them and a person checks by eye
        double[] moves = {position2, position1, position2, position1};
        for (double position : moves) {
            for (Servo servo : servos) {
                servo.setPosition(position);
            }
            ElapsedTime timer = new ElapsedTime();
            while (opModeIsActive() && timer.seconds() < SERVO_MOVE_SECONDS) {
                show("Watch the " + name + "...");
            }
        }

        if (pressAorB("Did the " + name + " move?", "A = yes,  B = no")) return PASS;
        return FAIL + " - " + name + " didn't move (" + servos[0].getConnectionInfo() + "). Check the servo wire and that the servo port has power";
    }

    // SENSOR TESTS
    // the color sensor and limelight are read all the time in showLiveSensors(), these tests just check if they ever saw something
    private String testColorSensor() {
        if (colorSensor == null) return notInConfig(COLOR_SENSOR);
        if (!pressAorB("Hold a ball right in front of the color sensor", "Watch 'Live color sensor' at the bottom", "A = done,  B = skip")) {
            return SKIPPED;
        }
        if (colorSensorSawBall) return PASS;
        return FAIL + " - never saw a ball. Check the sensor's wire and that the ball was close enough";
    }

    private String testLimelight() {
        if (limelight == null) return notInConfig(LIMELIGHT);
        if (!pressAorB("Point the limelight at an april tag", "Watch 'Live limelight' at the bottom", "A = done,  B = skip")) {
            return SKIPPED;
        }
        if (!limelight.isConnected()) return FAIL + " - limelight not connected. Check its USB cable and power";
        if (limelightSawTag) return PASS;
        return FAIL + " - never saw a tag. Check the lens is clean and the tag is in view";
    }

    private String testDrivetrain() {
        String[] motorNames = {FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, BACK_LEFT_WHEEL, BACK_RIGHT_WHEEL};
        DcMotorSimple.Direction[] directions = {
                Constants.drivetrainConfig.frontLeftDirection.get(),
                Constants.drivetrainConfig.frontRightDirection.get(),
                Constants.drivetrainConfig.backLeftDirection.get(),
                Constants.drivetrainConfig.backRightDirection.get()};
        DcMotorEx[] motors = new DcMotorEx[4];
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.tryGet(DcMotorEx.class, motorNames[i]);
            if (motors[i] == null) return notInConfig(motorNames[i]);
            motors[i].setDirection(directions[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (!pressAorB("Put the robot ON A BOX so the wheels are OFF THE FLOOR", "Each wheel will spin forward one at a time", "A = start,  B = skip")) {
            return SKIPPED;
        }

        String problems = "";
        for (int i = 0; i < 4; i++) {
            String motor = motorNames[i] + " (" + motors[i].getConnectionInfo() + ")";
            double highestAmps = 0;
            motors[i].setPower(DRIVE_POWER);
            ElapsedTime timer = new ElapsedTime();
            while (opModeIsActive() && timer.seconds() < DRIVE_SPIN_SECONDS) {
                highestAmps = Math.max(highestAmps, motors[i].getCurrent(CurrentUnit.AMPS));
                show("Wheel " + (i + 1) + "/4: watch " + motorNames[i] + "...");
            }
            motors[i].setPower(0);

            if (highestAmps < MIN_MOTOR_AMPS) {
                problems += motor + " got no power. Check its power wire is plugged in. ";
            } else if (!pressAorB("Did " + motorNames[i] + " spin FORWARD (top of the wheel moving toward the front)?", "A = yes,  B = no")) {
                problems += motor + " spins backwards. Flip its direction in Constants. ";
            }
        }

        if (problems.isEmpty()) return PASS;
        return FAIL + " - " + problems;
    }

    // PINPOINT TEST - you push and turn the robot by hand, and the pinpoint checks each move
    private String testPinpoint() {
        pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, PINPOINT);
        if (pinpoint == null) return notInConfig(PINPOINT);

        if (!pressAorB("Put the robot ON THE FLOOR on a tile line", "You will push and turn it BY HAND", "A = start,  B = skip")) {
            return SKIPPED;
        }

        show("Calibrating... DON'T touch the robot");
        // sets up the pinpoint exactly like pedro does in competition (pod type, offsets, directions from Constants)
        new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        pinpoint.update();
        // READY means both pods are detected and the pinpoint is working
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            return FAIL + " - pinpoint says " + pinpoint.getDeviceStatus() + ". Check both odometry pod wires";
        }

        String problems = "";

        if (!moveByHand("Step 1/3: Push the robot FORWARD about " + (int) PUSH_INCHES + " in")) return SKIPPED;
        problems += checkMove("Forward", forwardInches, leftInches, "x");

        if (!moveByHand("Step 2/3: Push the robot LEFT about " + (int) PUSH_INCHES + " in")) return SKIPPED;
        problems += checkMove("Left", leftInches, forwardInches, "y");

        if (!moveByHand("Step 3/3: Turn the robot LEFT (counterclockwise) about " + (int) TURN_DEGREES + " deg")) return SKIPPED;
        problems += checkTurn("Turn left", turnedDegrees);

        if (problems.isEmpty()) return PASS;
        return FAIL + " - " + problems;
    }

    private boolean moveByHand(String step) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        while (opModeIsActive()) {
            readPinpoint();
            if (gamepad1.aWasPressed()) return true;
            if (gamepad1.bWasPressed()) return false;
            show(step, "Then press A  (B = skip)", pinpointLine());
        }
        return false;
    }

    private void readPinpoint() {
        pinpoint.update();
        forwardInches = pinpoint.getPosX(DistanceUnit.INCH);
        leftInches = pinpoint.getPosY(DistanceUnit.INCH);
        turnedDegrees = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    private String pinpointLine() {
        return "Forward: " + round(forwardInches) + " in   Left: " + round(leftInches) + " in   Turned: " + round(turnedDegrees) + " deg";
    }

    private String checkMove(String move, double movedInches, double sidewaysInches, String pod) {
        if (Math.abs(movedInches) < POD_MIN_MOVE_INCHES && Math.abs(sidewaysInches) >= POD_MIN_MOVE_INCHES) {
            return move + ": pinpoint saw it go the wrong way. If you pushed it the right way, the x and y pod wires are swapped. ";
        }
        if (Math.abs(movedInches) < POD_MIN_MOVE_INCHES) {
            return move + ": " + pod + " pod didn't count. Check the pod's wire and that its wheel touches the floor. ";
        }
        if (movedInches < 0) {
            return move + ": counted backwards. Flip " + pod + "PodDirection. ";
        }
        return "";
    }

    private String checkTurn(String move, double turned) {
        if (Math.abs(turned) < TURN_MIN_DEGREES) {
            return move + ": heading didn't change. The pinpoint may be broken. ";
        }
        if (turned < 0) {
            return move + ": counted backwards. Check the pinpoint is mounted right side up. ";
        }
        return "";
    }

    // LIVE SENSORS - the color sensor and limelight readings shown at the bottom of the screen
    private void startLiveSensors() {
        if (isSelected("Color sensor") && hardwareMap.tryGet(RevColorSensorV3.class, COLOR_SENSOR) != null) {
            colorSensor = new ColorSensor(hardwareMap);
        }
        if (isSelected("Limelight")) {
            limelight = hardwareMap.tryGet(Limelight3A.class, LIMELIGHT);
            if (limelight != null) {
                limelight.pipelineSwitch(0); // pipeline 0 = april tag detection, same as competition
                limelight.start();
            }
        }
    }

    private void showLiveSensors() {
        if (colorSensor != null) {
            Element element = colorSensor.getDetectedColor();
            double distance = colorSensor.colorSensor.getDistance(DistanceUnit.CM);
            if (element != Element.NONE) colorSensorSawBall = true;
            telemetry.addData("Live color sensor", "sees " + element + ", " + round(distance) + " cm away");
        }

        if (limelight != null) {
            String tags = visibleTags();
            if (tags.isEmpty()) {
                telemetry.addData("Live limelight", "no tags");
            } else {
                limelightSawTag = true;
                telemetry.addData("Live limelight", "sees tags " + tags);
            }
        }
    }

    // returns the ids of every april tag the limelight sees right now, like "12 13 ", or "" if none
    private String visibleTags() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return "";

        String tags = "";
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            tags += tag.getFiducialId() + " ";
        }
        return tags;
    }

    // HELPERS - shared code used by the tests above, you usually don't need to change this
    private String notInConfig(String deviceName) {
        return FAIL + " - \"" + deviceName + "\" isn't in the robot config. Check the config on the Driver Hub and the spelling";
    }

    // shows the lines on screen and waits: returns true if A is pressed, false if B is pressed
    // "String... lines" means you can pass in as many lines of text as you want
    private boolean pressAorB(String... lines) {
        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) return true;
            if (gamepad1.bWasPressed()) return false;
            show(lines);
        }
        return false;
    }

    // draws the whole screen: title, instructions, all results so far, then the live sensors
    // this gets called over and over in every loop so the screen stays up to date
    private void show(String... lines) {
        telemetry.addLine(title);
        for (String line : lines) {
            telemetry.addLine(line);
        }

        telemetry.addLine();
        telemetry.addLine("---- RESULTS ----");
        for (int i = 0; i < tests.length; i++) {
            if (results[i] != null) telemetry.addData(tests[i], results[i]);
        }

        showLiveSensors();
        telemetry.update();
    }

    private boolean isSelected(String test) {
        for (int i = 0; i < tests.length; i++) {
            if (tests[i].equals(test)) return selected[i];
        }
        return false;
    }

    private void setAll(boolean value) {
        for (int i = 0; i < selected.length; i++) {
            selected[i] = value;
        }
    }

    // rounds to 1 decimal place, so 12.6348 shows as 12.6
    private double round(double value) {
        return Math.round(value * 10) / 10.0;
    }
}