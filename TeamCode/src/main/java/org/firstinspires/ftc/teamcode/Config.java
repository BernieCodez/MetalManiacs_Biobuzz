package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

import java.util.Set;

@Configurable
public class Config {
////////////////////////////////////////////////////////////////////////////////////////////////////
//                                       DRIVE TRAIN                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
    public static final String FRONT_LEFT_WHEEL = "frontLeft";
    public static final String FRONT_RIGHT_WHEEL = "frontRight";
    public static final String BACK_LEFT_WHEEL = "backLeft";
    public static final String BACK_RIGHT_WHEEL = "backRight";



////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     INTAKE/TRANSFER                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////
    public static final String INTAKE_TRANSFER = "intakeTransfer";
    public static final String GATE = "gate";
    public static double GATE_OPEN = 0.5;
    public static double GATE_CLOSE = 0;
    public static final String COLOR_SENSOR = "sensorColor";
    public enum Element{RED_NECTAR, BLUE_NECTAR, POLLEN, NONE};


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        FLYWHEELS                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
    public static final String LEFT_FLYWHEEL = "leftFlywheel";
    public static final String RIGHT_FLYWHEEL = "rightFlywheel";
    public static final String HOOD = "hood";
    public static final double HOOD_NECTAR = 0.5;
    public static final double HOOD_POLLEN = 0.25;

    //PIDF for flywheels
    public static double FLYWHEEL_KP = 0.0;
    public static double FLYWHEEL_KI = 0.0;
    public static double FLYWHEEL_KD = 0.0;
    public static double FLYWHEEL_KF = 0.0;

    public static double FLYWHEEL_VELOCITY_TOLERANCE = 20;
    //Slope calculator
    public static double FLYWHEEL_DISTANCE_SLOPE = 5;
    public static double FLYWHEEL_DISTANCE_INTERCEPT = 1000;



////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        AUTOAIM                                                 //
////////////////////////////////////////////////////////////////////////////////////////////////////

    public static boolean TUNING_ENABLED = true; //values from tuners get copied over

    //PIDF for position turret
    public static double AUTOAIM_P = 0.015; //make lower if robot is not spinning fast enough, make higher if robot is spinning too fast and overshooting
    public static double AUTOAIM_MAX_ROTATION_SPEED = 1; //0-1 1 is max


////////////////////////////////////////////////////////////////////////////////////////////////////
//                                      FLOWER WEDGE                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
    public static String RIGHT_WEDGE = "rightWedge";
    public static String LEFT_WEDGE = "leftWedge";
    public static double WEDGE_UP = 1;
    public static double WEDGE_DOWN = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                      LOCALIZATION                                              //
////////////////////////////////////////////////////////////////////////////////////////////////////
    public static final String PINPOINT = "pinpoint";
    public static final String LIMELIGHT = "limelight";
    public static final double RESULT_TIMEOUT_MS = 500;//for limelight

    public static final Pose LOCALIZATION_RESET_POSITION = new Pose(10.5, 10.5, Math.toRadians(90));

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                       HIVE POSES                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
    //GOAL POSITIONS FOR AUTO AIM

    public static final double HIVE_BOUNDARY = 72;

    public enum Alliance {
        RED(
                new Hive(//audience side
                        new Pose(57, 57),
                        Set.of(34, 35, 36, 37),
                        Set.of(35, 36)

                ),
                new Hive(//opposite audience
                        new Pose(57, 87),
                        Set.of(30, 31, 32, 33),
                        Set.of(31, 32)

                )

        ),

        BLUE(
                new Hive(//audience side
                        new Pose(87, 87),
                        Set.of(38, 39, 40, 41),
                        Set.of(39, 40)

                ),
                new Hive(//opposite audience
                        new Pose(87, 57),
                        Set.of(42, 43, 44, 45),
                        Set.of(43, 44)

                )
        );

        public final Hive TOP;
        public final Hive BOTTOM;

        Alliance(Hive top, Hive bottom) {
            this.TOP = top;
            this.BOTTOM = bottom;
        }

        public static class Hive {
            public final Pose pose;
            public final Set<Integer> allTagIds;
            public final Set<Integer> middleTagIds;

            Hive(Pose pose, Set<Integer> allTagIds, Set<Integer> middleTagIds) {
                this.pose = pose;
                this.allTagIds = allTagIds;
                this.middleTagIds = middleTagIds;

            }
        }
    }

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                        GROUPINGS                                               //
////////////////////////////////////////////////////////////////////////////////////////////////////
    public static final String PUSHBOT_ARM = "pushbotArm";
    public static final String[] MOTORS = {
            INTAKE_TRANSFER,
            RIGHT_FLYWHEEL,
            LEFT_FLYWHEEL,
            FRONT_LEFT_WHEEL,
            FRONT_RIGHT_WHEEL,
            BACK_LEFT_WHEEL,
            BACK_RIGHT_WHEEL,
            PUSHBOT_ARM

    };

    public static final String[] SERVOS = {
            HOOD,
            GATE
    };

    public class Hive {
    }
}