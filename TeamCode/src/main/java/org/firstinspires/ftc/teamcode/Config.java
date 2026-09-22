package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Config {

    public static final String INTAKE = "intake";
    public static final String FLYWHEEL = "flywheel";
    public static final String TURRET = "turret";

    public static final String FRONT_LEFT_WHEEL = "frontLeft";
    public static final String FRONT_RIGHT_WHEEL = "frontRight";
    public static final String BACK_LEFT_WHEEL = "backLeft";
    public static final String BACK_RIGHT_WHEEL = "backRight";

    public static final String[] MOTORS = {
            INTAKE,
            FLYWHEEL,
            FRONT_LEFT_WHEEL,
            FRONT_RIGHT_WHEEL,
            BACK_LEFT_WHEEL,
            BACK_RIGHT_WHEEL,
            TURRET
    };

    public static final String HOOD = "hood";
    public static final String GATE = "gate";

    public static final String[] SERVOS = {
            HOOD,
            GATE
    };

    public static final String PINPOINT = "pinpoint";

    public static boolean TUNING_ENABLED = true; //values from tuners get copied over

    //PIDF for turret
    public static double TURRET_POSITION_P = 0.0;

    public static double TURRET_VELOCITY_P = 0.0;
    public static double TURRET_VELOCITY_I = 0.0;
    public static double TURRET_VELOCITY_D = 0.0;
    public static double TURRET_VELOCITY_F = 0.0;

    public static int TURRET_POSITION_TOLERANCE = 10;


    //PIDF for flywheels
    public static double FLYWHEEL_KP = 0.0;
    public static double FLYWHEEL_KI = 0.0;
    public static double FLYWHEEL_KD = 0.0;
    public static double FLYWHEEL_KF = 0.0;

    public static double FLYWHEEL_VELOCITY_TOLERANCE = 20;
}