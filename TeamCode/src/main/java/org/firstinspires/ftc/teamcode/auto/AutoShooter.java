package org.firstinspires.ftc.teamcode.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controllers.OuttakeController;

@Configurable
public class AutoShooter {

    public static long SHOT_OPEN_MS = 1400;
    public static long SHOT_GAP_MS = 400;
    public static int GATE_OPENS_PER_SHOOT = 1;
    public static boolean NECTAR_GATE_ONLY = false;
    public static double SHOT_FLYWHEEL_VELOCITY = 1150;
    public static boolean RAMP_ENABLED = true;
    public static double RAMP_START_MS = 30;
    public static double RAMP_DURATION_MS = 5000;
    public static double RAMP_AMOUNT = 300;

    private final OuttakeController outtake;
    private final ElapsedTime gateTimer = new ElapsedTime();
    private boolean gateOpen = false;

    public AutoShooter(OuttakeController outtake) {
        this.outtake = outtake;
    }

    public void openGate() {
        if (NECTAR_GATE_ONLY) {
            outtake.fireNectar();
        } else {
            outtake.fire();
        }
        gateOpen = true;
        gateTimer.reset();
    }

    public void closeGate() {
        outtake.close();
        gateOpen = false;
    }

    public double targetVelocity() {
        if (!RAMP_ENABLED || !gateOpen) return SHOT_FLYWHEEL_VELOCITY;
        double msIntoRamp = gateTimer.milliseconds() - RAMP_START_MS;
        if (msIntoRamp <= 0) return SHOT_FLYWHEEL_VELOCITY;
        double fraction = 1;
        if (RAMP_DURATION_MS > 0) {
            fraction = Math.min(1, msIntoRamp / RAMP_DURATION_MS);
        }
        return SHOT_FLYWHEEL_VELOCITY + RAMP_AMOUNT * fraction;
    }

    public void update() {
        outtake.flywheel.setVelocity(targetVelocity());
        outtake.flywheel.update();
    }
}
