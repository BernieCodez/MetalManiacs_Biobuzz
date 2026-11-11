package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.Config.FLYWHEEL_VELOCITY_TOLERANCE;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.Plugin;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.Auto;
import org.firstinspires.ftc.teamcode.hardware.Flywheel;
import org.firstinspires.ftc.teamcode.hardware.Gate;
import org.firstinspires.ftc.teamcode.hardware.IntakeTransfer;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

import java.util.ArrayList;
import java.util.List;

@Configurable
@TeleOp(name = "Gate Timing Tuner", group = "A - Test")
public class GateTimingTuner extends LinearOpMode {

    public static boolean FIRE = false;
    public static int SHOTS_PER_PRESS = Auto.GATE_OPENS_PER_SHOOT;
    public static double SHOT_OPEN_MS = Auto.SHOT_OPEN_MS;
    public static double SHOT_GAP_MS = Auto.SHOT_GAP_MS;
    public static double FLYWHEEL_VELOCITY = Auto.SHOT_FLYWHEEL_VELOCITY;
    public static double INTAKE_POWER = 1;
    public static boolean NECTAR_GATE = Auto.NECTAR_GATE_ONLY;

    public static boolean RAMP_ENABLED = Auto.RAMP_ENABLED;
    public static double RAMP_START_MS = Auto.RAMP_START_MS;
    public static double RAMP_DURATION_MS = Auto.RAMP_DURATION_MS;
    public static double RAMP_AMOUNT = Auto.RAMP_AMOUNT;

    public static double OPEN_MS_STEP = 100;

    private enum Phase { IDLE, OPEN, GAP }

    private static class ShotResult {
        double startVelocity;
        double lowestVelocity = Double.MAX_VALUE;
        double recoveryMs = -1;
    }

    private final TelemetryManager panels = Plugin.INSTANCE.getManager();

    private Flywheel flywheel;
    private Gate gate;
    private IntakeTransfer intake;

    private Phase phase = Phase.IDLE;
    private int shotsLeft = 0;
    private double burstOpenMs;
    private double burstGapMs;
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private final List<ShotResult> lastBurst = new ArrayList<>();
    private ShotResult currentShot;

    private void openGate() {
        if (NECTAR_GATE) gate.openNectar();
        else gate.open();
    }

    private void startShot(double velocity) {
        currentShot = new ShotResult();
        currentShot.startVelocity = velocity;
        lastBurst.add(currentShot);
        openGate();
        phase = Phase.OPEN;
        phaseTimer.reset();
    }

    private double targetVelocity() {
        if (!RAMP_ENABLED || phase != Phase.OPEN) return FLYWHEEL_VELOCITY;
        double msIntoRamp = phaseTimer.milliseconds() - RAMP_START_MS;
        if (msIntoRamp <= 0) return FLYWHEEL_VELOCITY;
        double fraction = 1;
        if (RAMP_DURATION_MS > 0) {
            fraction = Math.min(1, msIntoRamp / RAMP_DURATION_MS);
        }
        return FLYWHEEL_VELOCITY + RAMP_AMOUNT * fraction;
    }

    private void step(double velocity) {
        if (phase == Phase.IDLE) {
            if (FIRE) {
                lastBurst.clear();
                shotsLeft = Math.max(1, SHOTS_PER_PRESS);
                burstOpenMs = Math.max(0, SHOT_OPEN_MS);
                burstGapMs = Math.max(0, SHOT_GAP_MS);
                startShot(velocity);
            }
            return;
        }

        currentShot.lowestVelocity = Math.min(currentShot.lowestVelocity, velocity);

        if (phase == Phase.OPEN && phaseTimer.milliseconds() >= burstOpenMs) {
            gate.close();
            phase = Phase.GAP;
            phaseTimer.reset();
        } else if (phase == Phase.GAP) {
            boolean recovered = Math.abs(FLYWHEEL_VELOCITY - velocity) < FLYWHEEL_VELOCITY_TOLERANCE;
            if (recovered && currentShot.recoveryMs < 0) {
                currentShot.recoveryMs = phaseTimer.milliseconds();
            }
            if (phaseTimer.milliseconds() >= burstGapMs) {
                shotsLeft--;
                if (shotsLeft > 0) {
                    startShot(velocity);
                } else {
                    phase = Phase.IDLE;
                    FIRE = false;
                }
            }
        }
    }

    private String verdict() {
        if (lastBurst.isEmpty()) return "Press X / Square to fire a burst";
        double worst = 0;
        for (ShotResult r : lastBurst) {
            if (r.recoveryMs < 0) return "Flywheel did NOT recover before the next shot: raise SHOT_GAP_MS";
            worst = Math.max(worst, r.recoveryMs);
        }
        return String.format("Flywheel recovered within %.0f ms: SHOT_GAP_MS can go down to about %.0f", worst, worst + 50);
    }

    @Override
    public void runOpMode() {
        flywheel = new Flywheel(hardwareMap);
        gate = new Gate(hardwareMap);
        intake = new IntakeTransfer(hardwareMap);
        FIRE = false;

        telemetry.addLine("Gate Timing Tuner");
        telemetry.addLine("X / Square: fire");
        telemetry.addLine("Dpad up/down: shots per press");
        telemetry.addLine("Dpad right/left: gate open time");
        telemetry.addLine("Triangle / Y: toggle ramp");
        telemetry.addLine("Circle / B: abort burst");
        telemetry.update();

        RumbleGamepad gp = new RumbleGamepad(gamepad1);

        waitForStart();
        gate.close();

        while (opModeIsActive()) {
            gp.update();
            if (gp.wasJustPressed(RumbleGamepad.Button.X) || gp.wasJustPressed(RumbleGamepad.Button.SQUARE)) {
                FIRE = true;
            }
            if (gp.wasJustPressed(RumbleGamepad.Button.DPAD_UP)) SHOTS_PER_PRESS++;
            if (gp.wasJustPressed(RumbleGamepad.Button.DPAD_DOWN)) SHOTS_PER_PRESS = Math.max(1, SHOTS_PER_PRESS - 1);
            if (gp.wasJustPressed(RumbleGamepad.Button.DPAD_RIGHT)) SHOT_OPEN_MS += OPEN_MS_STEP;
            if (gp.wasJustPressed(RumbleGamepad.Button.DPAD_LEFT)) SHOT_OPEN_MS = Math.max(0, SHOT_OPEN_MS - OPEN_MS_STEP);
            if (gp.wasJustPressed(RumbleGamepad.Button.Y) || gp.wasJustPressed(RumbleGamepad.Button.TRIANGLE)) {
                RAMP_ENABLED = !RAMP_ENABLED;
            }
            if ((gp.wasJustPressed(RumbleGamepad.Button.B) || gp.wasJustPressed(RumbleGamepad.Button.CIRCLE)) && phase != Phase.IDLE) {
                gate.close();
                phase = Phase.IDLE;
                shotsLeft = 0;
                FIRE = false;
            }

            double target = targetVelocity();
            flywheel.setVelocity(target);
            intake.setPower(INTAKE_POWER);

            double velocity = flywheel.getVelocity();
            step(velocity);

            panels.addData("flywheel velocity", velocity);
            panels.addData("target velocity", target);
            panels.addData("gate open", phase == Phase.OPEN ? 1 : 0);

            telemetry.addData("Phase", phase.name());
            if (phase == Phase.OPEN) {
                telemetry.addData("Gate open for (ms)", "%.0f", phaseTimer.milliseconds());
            } else {
                telemetry.addData("Gate open for (ms)", "-");
            }
            telemetry.addData("Shots left", shotsLeft);
            telemetry.addData("SHOTS_PER_PRESS", SHOTS_PER_PRESS);
            telemetry.addData("SHOT_OPEN_MS", "%.0f", SHOT_OPEN_MS);
            telemetry.addData("SHOT_GAP_MS", "%.0f", SHOT_GAP_MS);
            telemetry.addData("Burst time (ms)", "%.0f", SHOTS_PER_PRESS * (SHOT_OPEN_MS + SHOT_GAP_MS));
            telemetry.addData("Target velocity", "%.0f", target);
            if (RAMP_ENABLED) {
                telemetry.addData("Ramp", "+%.0f starting %.0f ms after open, over %.0f ms", RAMP_AMOUNT, RAMP_START_MS, RAMP_DURATION_MS);
            } else {
                telemetry.addData("Ramp", "OFF");
            }
            telemetry.addLine("");
            for (int i = 0; i < lastBurst.size(); i++) {
                ShotResult r = lastBurst.get(i);
                double low = r.lowestVelocity;
                if (low == Double.MAX_VALUE) low = r.startVelocity;
                String recovered = "NO";
                if (r.recoveryMs >= 0) recovered = String.format("%.0f ms", r.recoveryMs);
                telemetry.addLine(String.format("Shot %d: start %.0f  low %.0f  recovered %s", i + 1, r.startVelocity, low, recovered));
            }
            telemetry.addLine("");
            telemetry.addLine(verdict());
            telemetry.addLine("When happy, copy SHOT_OPEN_MS and SHOT_GAP_MS into Auto.java");

            panels.update(telemetry);
        }

        gate.close();
        flywheel.stop();
        intake.stop();
    }
}
