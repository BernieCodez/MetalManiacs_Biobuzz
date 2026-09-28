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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;
import java.util.List;

@TeleOp(name = "Pushbot Code", group = "C - Outreach")
public class PushbotDriveCode extends OpMode {
    public RumbleGamepad driver;
    List<LynxModule> allHubs;

    // Hardware - expansion hub motor port 0/1/2, servo port 0/1
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor armMotor;
    Servo leftClaw;
    Servo rightClaw;
    Servo tail;

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

    // --- Dance mode fields ---
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

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftClaw.setPosition(LEFT_HAND_CLOSED);
        rightClaw.setPosition(HAND_CLOSED);
        driver.light(255, 0, 0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        driver.update();

        double forward = -driver.leftY();
        double turn = driver.rightX();

        // Dance mode toggle logic
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
            // Adjust driver speed
            if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_UP)) {
                driveSpeed = Math.min(1.0, driveSpeed + 0.1);
            } else if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_DOWN)) {
                driveSpeed = Math.max(0.1, driveSpeed - 0.1);
            }

            // Normal Drive
            leftPower = Range.clip(forward - turn, -1.0, 1.0) * driveSpeed;
            rightPower = Range.clip(forward + turn, -1.0, 1.0) * driveSpeed;
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Normal Claw
            if (driver.wasJustPressed(RumbleGamepad.Button.A)) {
                leftClaw.setPosition(LEFT_HAND_OPEN);
                rightClaw.setPosition(HAND_OPEN);
            } else if (driver.wasJustPressed(RumbleGamepad.Button.B)) {
                leftClaw.setPosition(LEFT_HAND_CLOSED);
                rightClaw.setPosition(HAND_CLOSED);
            }
        }

        // Arm control runs independently, allowing movement during a dance sequence
        int currentArmPos = armMotor.getCurrentPosition();
        if (driver.isDown(RumbleGamepad.Button.RIGHT_BUMPER)) {
            armMotor.setPower(1);
        } else if (driver.isDown(RumbleGamepad.Button.LEFT_BUMPER)) {
            armMotor.setPower(-1);
        } else {
            armMotor.setPower(0);
        }

        // Tail control runs independently
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

        // Telemetry
        telemetry.addData("Dance Status", activeDance == Dance.NONE ? "Manual" : "DANCING: " + activeDance + " Step: " + danceStep);
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Arm Position", currentArmPos);
        telemetry.addData("Left Claw", leftClaw.getPosition());
        telemetry.addData("Right Claw", rightClaw.getPosition());
        telemetry.update();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armMotor.setPower(0);
    }

    // --- Helper Dance Methods ---
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
}