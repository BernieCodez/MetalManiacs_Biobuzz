package org.firstinspires.ftc.teamcode.teleop;

/**
 * <DRIVER MANUAL>
 *
 * --DRIVER CONTROLS--
 *
 * [MOVEMENT]
 * LEFT STICK Y = forward / backward
 * RIGHT STICK X = turn
 * DPAD UP = drive speed up
 * DPAD DOWN = drive speed down
 * LEFT STICK BUTTON = toggle "Spin Cycle" dance (press again, or move a stick, to cancel)
 *
 * [ARM]
 * RIGHT BUMPER (hold) = raise arm
 * LEFT BUMPER (hold) = lower arm
 *
 * [HAND / GRIPPER]
 * A = open gripper
 * B = close gripper
 */

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Pushbot Code", group = "C - Outreach")
public class PushbotDriveCode extends OpMode {
    public RumbleGamepad driver;
    List<LynxModule> allHubs;

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor armMotor;
    Servo leftClaw;
    Servo rightClaw;
    Servo tail;

    GoBildaPrismDriver prism;
    TouchSensor touchSensor;

    boolean isTailWagging = false;
    double driveSpeed = 1.0;
    ElapsedTime tailTimer = new ElapsedTime();
    boolean tailOn = false;

    public static final double HAND_OPEN = 0;
    public static final double HAND_CLOSED = 0.2;
    public static final double LEFT_HAND_OPEN = 0.2;
    public static final double LEFT_HAND_CLOSED = 0;
    public static final double TAIL_LEFT = 0.3;
    public static final double TAIL_RIGHT = 0.7;

    private final ArrayList<Object> patterns = new ArrayList<>();
    private int currentPatternIndex = 0;
    private boolean lastTouchState = false;
    private boolean isLedInitialized = false;

    private PrismAnimations.Rainbow rainbowAnimation;
    private boolean wasDancingLastFrame = false;

    private enum Dance { NONE, SPIN_CYCLE }
    private static final double STICK_CANCEL_THRESHOLD = 0.2;
    private Dance activeDance = Dance.NONE;
    private int danceStep = 0;
    private int lastAppliedDanceStep = -1;
    private final ElapsedTime danceTimer = new ElapsedTime();

    private static class DanceStep {
        final double leftPower;
        final double rightPower;
        final double duration;
        final Boolean clawOpen;

        DanceStep(double leftPower, double rightPower, double duration) {
            this(leftPower, rightPower, duration, null);
        }

        DanceStep(double leftPower, double rightPower, double duration, Boolean clawOpen) {
            this.leftPower = leftPower;
            this.rightPower = rightPower;
            this.duration = duration;
            this.clawOpen = clawOpen;
        }
    }

    private final DanceStep[] SPIN_CYCLE_DANCE = {
            new DanceStep(0.6, -0.6, 0.6),
            new DanceStep(-0.6, 0.6, 0.6),
            new DanceStep(0.6, -0.6, 0.6),
            new DanceStep(-0.6, 0.6, 0.6),
            new DanceStep(0, 0, 0.2, true),
            new DanceStep(0, 0, 0.2, false),
            new DanceStep(0, 0, 0.2, true),
            new DanceStep(0, 0, 0.2, false),
    };

    @Override
    public void init() {
        driver = new RumbleGamepad(gamepad1);
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        tail = hardwareMap.get(Servo.class, "tail");

        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftClaw.setPosition(LEFT_HAND_CLOSED);
        rightClaw.setPosition(HAND_CLOSED);
        driver.light(255, 0, 0);

        PrismAnimations.Solid solidGreen   = new PrismAnimations.Solid(Color.GREEN);
        PrismAnimations.Solid solidPurple  = new PrismAnimations.Solid(Color.PURPLE);
        PrismAnimations.Solid solidPink    = new PrismAnimations.Solid(Color.PINK);
        PrismAnimations.Solid solidMagenta = new PrismAnimations.Solid(Color.MAGENTA);
        PrismAnimations.Solid solidCyan    = new PrismAnimations.Solid(Color.CYAN);
        PrismAnimations.Solid solidTeal    = new PrismAnimations.Solid(Color.TEAL);
        PrismAnimations.Solid solidOrange  = new PrismAnimations.Solid(Color.ORANGE);
        PrismAnimations.Solid solidRed     = new PrismAnimations.Solid(Color.RED);
        PrismAnimations.Solid solidYellow  = new PrismAnimations.Solid(Color.YELLOW);
        PrismAnimations.Solid solidOlive   = new PrismAnimations.Solid(Color.OLIVE);
        PrismAnimations.Solid solidBlue    = new PrismAnimations.Solid(Color.BLUE);
        PrismAnimations.Solid solidWhite   = new PrismAnimations.Solid(Color.WHITE);

        rainbowAnimation = new PrismAnimations.Rainbow();

        int brightness = 50;
        int startIdx = 0;
        int stopIdx = 12;

        solidGreen.setBrightness(brightness);   solidGreen.setStartIndex(startIdx);   solidGreen.setStopIndex(stopIdx);
        solidPurple.setBrightness(brightness);  solidPurple.setStartIndex(startIdx);  solidPurple.setStopIndex(stopIdx);
        solidPink.setBrightness(brightness);    solidPink.setStartIndex(startIdx);    solidPink.setStopIndex(stopIdx);
        solidMagenta.setBrightness(brightness); solidMagenta.setStartIndex(startIdx); solidMagenta.setStopIndex(stopIdx);
        solidCyan.setBrightness(brightness);    solidCyan.setStartIndex(startIdx);    solidCyan.setStopIndex(stopIdx);
        solidTeal.setBrightness(brightness);    solidTeal.setStartIndex(startIdx);    solidTeal.setStopIndex(stopIdx);
        solidOrange.setBrightness(brightness);  solidOrange.setStartIndex(startIdx);  solidOrange.setStopIndex(stopIdx);
        solidRed.setBrightness(brightness);     solidRed.setStartIndex(startIdx);     solidRed.setStopIndex(stopIdx);
        solidYellow.setBrightness(brightness);  solidYellow.setStartIndex(startIdx);  solidYellow.setStopIndex(stopIdx);
        solidOlive.setBrightness(brightness);   solidOlive.setStartIndex(startIdx);   solidOlive.setStopIndex(stopIdx);
        solidBlue.setBrightness(brightness);    solidBlue.setStartIndex(startIdx);    solidBlue.setStopIndex(stopIdx);
        solidWhite.setBrightness(brightness);   solidWhite.setStartIndex(startIdx);   solidWhite.setStopIndex(stopIdx);

        rainbowAnimation.setBrightness(brightness); rainbowAnimation.setStartIndex(startIdx); rainbowAnimation.setStopIndex(stopIdx);

        patterns.add(solidGreen);
        patterns.add(solidPurple);
        patterns.add(solidPink);
        patterns.add(solidMagenta);
        patterns.add(solidCyan);
        patterns.add(solidTeal);
        patterns.add(solidOrange);
        patterns.add(solidRed);
        patterns.add(solidYellow);
        patterns.add(solidOlive);
        patterns.add(solidBlue);
        patterns.add(solidWhite);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        driver.update();

        boolean isCurrentlyDancing = (activeDance != Dance.NONE);

        if (!isLedInitialized) {
            updatePrismLED(patterns.get(currentPatternIndex));
            isLedInitialized = true;
        }

        boolean currentTouchState = touchSensor.isPressed();

        if (!isCurrentlyDancing && currentTouchState && !lastTouchState) {
            currentPatternIndex++;
            if (currentPatternIndex >= patterns.size()) {
                currentPatternIndex = 0;
            }
            updatePrismLED(patterns.get(currentPatternIndex));
        }
        lastTouchState = currentTouchState;

        if (isCurrentlyDancing && !wasDancingLastFrame) {
            updatePrismLED(rainbowAnimation);
        } else if (!isCurrentlyDancing && wasDancingLastFrame) {
            updatePrismLED(patterns.get(currentPatternIndex));
        }
        wasDancingLastFrame = isCurrentlyDancing;

        double forward = -driver.leftY();
        double turn = driver.rightX();

        if (driver.wasJustPressed(RumbleGamepad.Button.LEFT_STICK)) {
            toggleDance(Dance.SPIN_CYCLE);
            gamepad1.rumble(0.5, 0.5, 200);
        } else if (activeDance != Dance.NONE
                && (Math.abs(forward) > STICK_CANCEL_THRESHOLD || Math.abs(turn) > STICK_CANCEL_THRESHOLD)) {
            activeDance = Dance.NONE;
        }

        double leftPower;
        double rightPower;

        if (activeDance != Dance.NONE) {
            runDance();
            leftPower = leftDrive.getPower();
            rightPower = rightDrive.getPower();
        } else {
            if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_UP)) {
                driveSpeed = Math.min(1.0, driveSpeed + 0.1);
            } else if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_DOWN)) {
                driveSpeed = Math.max(0.1, driveSpeed - 0.1);
            }

            leftPower = Range.clip(forward - turn, -1.0, 1.0) * driveSpeed;
            rightPower = Range.clip(forward + turn, -1.0, 1.0) * driveSpeed;
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            if (driver.wasJustPressed(RumbleGamepad.Button.A)) {
                leftClaw.setPosition(LEFT_HAND_OPEN);
                rightClaw.setPosition(HAND_OPEN);
            } else if (driver.wasJustPressed(RumbleGamepad.Button.B)) {
                leftClaw.setPosition(LEFT_HAND_CLOSED);
                rightClaw.setPosition(HAND_CLOSED);
            }
        }
        int currentArmPos = armMotor.getCurrentPosition();
        if (driver.isDown(RumbleGamepad.Button.RIGHT_BUMPER)) {
            armMotor.setPower(1);
        } else if (driver.isDown(RumbleGamepad.Button.LEFT_BUMPER)) {
            armMotor.setPower(-1);
        } else {
            armMotor.setPower(0);
        }
        if (driver.wasJustPressed(RumbleGamepad.Button.Y)) {
            isTailWagging = !isTailWagging;
            tailTimer.reset();
        }
        if (isTailWagging) {
            if (tailTimer.milliseconds() >= 300) {
                tailOn = !tailOn;
                tail.setPosition(tailOn ? TAIL_LEFT : TAIL_RIGHT);
                tailTimer.reset();
            }
        } else {
            tail.setPosition(0.5);
        }
        telemetry.addData("Dance Status", activeDance == Dance.NONE ? "Manual" : "DANCING: " + activeDance + " Step: " + danceStep);
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Arm Position", currentArmPos);
        telemetry.addData("Left Claw", leftClaw.getPosition());
        telemetry.addData("Right Claw", rightClaw.getPosition());
        telemetry.update();
    }
    @Override
    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armMotor.setPower(0);
    }
    private void toggleDance(Dance dance) {
        if (activeDance == dance) {
            activeDance = Dance.NONE;
        } else {
            activeDance = dance;
            danceStep = 0;
            lastAppliedDanceStep = -1;
            danceTimer.reset();
        }
    }
    private void runDance() {
        DanceStep[] sequence = (activeDance == Dance.SPIN_CYCLE) ? SPIN_CYCLE_DANCE : null;
        if (sequence == null) return;
        if (danceStep >= sequence.length) {
            activeDance = Dance.NONE;
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            return;
        }
        DanceStep step = sequence[danceStep];
        if (danceStep != lastAppliedDanceStep) {
            leftDrive.setPower(step.leftPower);
            rightDrive.setPower(step.rightPower);
            if (step.clawOpen != null) {
                if (step.clawOpen) {
                    leftClaw.setPosition(LEFT_HAND_OPEN);
                    rightClaw.setPosition(HAND_OPEN);
                } else {
                    leftClaw.setPosition(LEFT_HAND_CLOSED);
                    rightClaw.setPosition(HAND_CLOSED);
                }
            }
            lastAppliedDanceStep = danceStep;
        }
        if (danceTimer.seconds() >= step.duration) {
            danceStep++;
            danceTimer.reset();
        }
    }
    private void updatePrismLED(Object patternObj) {
        if (patternObj instanceof PrismAnimations.Solid) {
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, (PrismAnimations.Solid) patternObj);
        } else if (patternObj instanceof PrismAnimations.Rainbow) {
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, (PrismAnimations.Rainbow) patternObj);
        }
    }
}