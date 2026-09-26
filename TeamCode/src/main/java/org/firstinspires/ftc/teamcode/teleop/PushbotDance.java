/**
 *  <DRIVER MANUAL>
 * ©JavaDude
 *  --DRIVER CONTROLS--
 *
 *  [MOVEMENT]
 *  LEFT STICK Y = forward / backward
 *  RIGHT STICK X = turn
 *  DPAD UP       = drive speed up
 *  DPAD DOWN     = drive speed down
 *  LEFT TRIGGER  = Slow / Precision Mode (0.3x speed)
 *  RIGHT TRIGGER = Turbo Mode (1.0x speed)
 *  LEFT STICK BUTTON  = toggle "Spin Cycle" dance (press again, or move a stick, to cancel)
 *
 *  [HAND / GRIPPER]
 *  A = open gripper
 *  B = close gripper
 */
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

import java.util.List;

@TeleOp(name = "PushbotDance", group = "A - TeleOP")
public class PushbotDance extends OpMode {

    public RumbleGamepad driver;
    List<LynxModule> allHubs;

    // Hardware - expansion hub motor port 0/1, servo port 0/1
    DcMotor leftDrive;
    DcMotor rightDrive;
    Servo leftClaw;
    Servo rightClaw;
    VoltageSensor voltageSensor;

    double driveSpeed = 1.0;
    double lastLeftPower = 0;
    double lastRightPower = 0;
    ElapsedTime loopTimer = new ElapsedTime();

    public static double DRIVE_SLEW_RATE = 3.0; // Power units per second

    public static final double DRIVE_SPEED_STEP = 0.1;
    public static final double DRIVE_SPEED_MIN = 0.1;
    public static final double DRIVE_SPEED_MAX = 1.0;

    public static final double HAND_OPEN = 0;
    public static final double HAND_CLOSED = 0.2;

    public static final double LEFT_HAND_OPEN = 0.2;
    public static final double LEFT_HAND_CLOSED = 0;

    // --- Dance mode ---

    private enum Dance { NONE, SPIN_CYCLE }

    private static final double STICK_CANCEL_THRESHOLD = 0.2;

    private Dance activeDance = Dance.NONE;
    private int danceStep = 0;
    private int lastAppliedDanceStep = -1;
    private final ElapsedTime danceTimer = new ElapsedTime();

    /** One beat of a dance: drive powers to hold, how long to hold them, and an optional claw cue. */
    private static class DanceStep {
        final double leftPower;
        final double rightPower;
        final double duration;
        final Boolean clawOpen;  // null = leave unchanged

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

    // "Spin Cycle" - alternating spins finished off with a gripper snap
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
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftClaw.setPosition(LEFT_HAND_CLOSED);
        rightClaw.setPosition(HAND_CLOSED);

        loopTimer.reset();

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

        double dt = loopTimer.seconds();
        loopTimer.reset();

        double forward = driver.leftY();
        double turn = driver.rightX();

        // Dance mode toggle - re-pressing the stick button, or nudging a stick, cancels it
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
            // Drive speed adjust
            if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_UP)) {
                driveSpeed = Math.min(DRIVE_SPEED_MAX, driveSpeed + DRIVE_SPEED_STEP);
            } else if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_DOWN)) {
                driveSpeed = Math.max(DRIVE_SPEED_MIN, driveSpeed - DRIVE_SPEED_STEP);
            }

            // Apply Exponential Curve to joysticks for better precision
            forward = forward * Math.abs(forward);
            turn = turn * Math.abs(turn);

            // Trigger Scaling (Slow/Turbo)
            double currentScale = driveSpeed;
            if (driver.leftTrigger() > 0.1) {
                currentScale = 0.3; // Precision Mode
            } else if (driver.rightTrigger() > 0.1) {
                currentScale = 1.0; // Turbo Mode
            }

            // Drive
            double targetLeft = Range.clip(forward - turn, -1.0, 1.0) * currentScale;
            double targetRight = Range.clip(forward + turn, -1.0, 1.0) * currentScale;

            // Voltage Compensation
            double voltage = voltageSensor.getVoltage();
            double voltageComp = 12.0 / Math.max(voltage, 1.0);
            targetLeft *= voltageComp;
            targetRight *= voltageComp;

            // Slew Rate Limiter
            double maxChange = DRIVE_SLEW_RATE * dt;
            leftPower = lastLeftPower + Range.clip(targetLeft - lastLeftPower, -maxChange, maxChange);
            rightPower = lastRightPower + Range.clip(targetRight - lastRightPower, -maxChange, maxChange);

            lastLeftPower = leftPower;
            lastRightPower = rightPower;

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Hand / Gripper
            if (driver.wasJustPressed(RumbleGamepad.Button.A)) {
                leftClaw.setPosition(LEFT_HAND_OPEN);
                rightClaw.setPosition(HAND_OPEN);
            } else if (driver.wasJustPressed(RumbleGamepad.Button.B)) {
                leftClaw.setPosition(LEFT_HAND_CLOSED);
                rightClaw.setPosition(HAND_CLOSED);
            }
        }

        // Displays important information for driver
        telemetry.addLine("--- DRIVE ---");
        telemetry.addData("Voltage", "%.2fV (Comp: %.2f)", voltageSensor.getVoltage(), 12.0 / Math.max(voltageSensor.getVoltage(), 1.0));
        telemetry.addData("Speed Scale", "%.2f", driveSpeed);
        telemetry.addData("Forward / Turn", "%.2f / %.2f", forward, turn);
        telemetry.addData("Powers (L/R)", "%.2f / %.2f", leftPower, rightPower);

        telemetry.addLine("--- CLAW ---");
        telemetry.addData("Claw (L/R)", "%.2f / %.2f", leftClaw.getPosition(), rightClaw.getPosition());

        telemetry.addLine("--- STATUS ---");
        if (activeDance != Dance.NONE) {
            telemetry.addData("Dance", "%s (Step %d, %.1fs)", activeDance, danceStep, danceTimer.seconds());
        } else {
            telemetry.addData("Dance", "None");
        }
        telemetry.update();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
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

    /** Advances the active dance's step sequencer by one loop tick without blocking. */
    private void runDance() {
        DanceStep[] sequence = SPIN_CYCLE_DANCE;

        if (danceStep >= sequence.length) {
            activeDance = Dance.NONE;
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            return;
        }

        DanceStep step = sequence[danceStep];

        if (danceStep != lastAppliedDanceStep) {
            lastAppliedDanceStep = danceStep;
            danceTimer.reset();

            if (step.clawOpen != null) {
                leftClaw.setPosition(step.clawOpen ? LEFT_HAND_OPEN : LEFT_HAND_CLOSED);
                rightClaw.setPosition(step.clawOpen ? HAND_OPEN : HAND_CLOSED);
            }
        }

        leftDrive.setPower(step.leftPower);
        rightDrive.setPower(step.rightPower);

        if (danceTimer.seconds() >= step.duration) {
            danceStep++;
        }
    }

    @Override
    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
