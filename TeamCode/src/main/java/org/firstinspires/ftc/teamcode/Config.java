package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import java.util.Set;

@Configurable
public class Config {
    public static final String TRANSFER = "transfer";
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
            GATE,
            BACK_RIGHT_WHEEL,
            TRANSFER,

    };

    public static final String PINPOINT = "pinpoint";
    public static final String LIMELIGHT = "limelight";
    public static final double RESULT_TIMEOUT_MS = 500;

    public static boolean TUNING_ENABLED = true; //values from tuners get copied over

    //PIDF for turret
    public static double TURRET_POSITION_P = 0.0;

    public static double TURRET_VELOCITY_P = 0.0;
    public static double TURRET_VELOCITY_I = 0.0;
    public static double TURRET_VELOCITY_D = 0.0;
    public static double TURRET_VELOCITY_F = 0.0;

    public static int TURRET_POSITION_TOLERANCE = 10;

    //OTHER TURRET SHENANIGANS
    public final static double TURRET_CENTER_TICKS = 750;
    public final static double TURRET_TICKS_PER_DEGREE = 4.1667;
    //encoder positions
    public static final int MIN_TURRET_POSITION = 0;
    public static final int MAX_TURRET_POSITION = 1500; //placeholders beware!


    //PIDF for flywheels
    public static double FLYWHEEL_KP = 0.0;
    public static double FLYWHEEL_KI = 0.0;
    public static double FLYWHEEL_KD = 0.0;
    public static double FLYWHEEL_KF = 0.0;

    public static double FLYWHEEL_VELOCITY_TOLERANCE = 20;

    //GOAL POSITIONS FOR AUTO AIM
    public static final PoseFactory poseFactory = PoseFactory.degrees();
    public static final double HIVE_BOUNDARY = 72;
    public enum Hive {
        RED_TOP(
                poseFactory.of(57, 87, 0),
                Set.of(30, 31, 32, 33)
        ),

        RED_BOTTOM(
                poseFactory.of(57, 57, 0),
                Set.of(34, 35, 36, 37)
        ),

        BLUE_TOP(
                poseFactory.of(87,87,0),
                Set.of(38, 39, 40, 41)
        ),

        BLUE_BOTTOM(
                poseFactory.of(87,57,0),
                Set.of(42, 43, 44, 45)
        );

        public final Pose pose;
        public final Set<Integer> tagIds;

        Hive(Pose pose, Set<Integer> tagIds) {
            this.pose = pose;
            this.tagIds = tagIds;
        }
    }
    public static final Set<Integer> MIDDLE_TAG_IDS = Set.of(31, 32, 35, 36, 39, 40, 43, 44);
}