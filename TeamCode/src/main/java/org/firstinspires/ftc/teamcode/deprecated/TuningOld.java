//package org.firstinspires.ftc.teamcode.deprecated;
//
//import com.pedropathing.algorithm.Foresight;
//import com.pedropathing.revhub.drivetrains.Mecanum;
//import com.pedropathing.revhub.localizers.PinpointLocalizer;
//import com.pedropathing.tuning.autotune.Procedure;
//import com.pedropathing.tuning.autotune.Tuner;
//
//import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
//import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
//import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
//import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
//@Deprecated(since = "v1.34.0")
//public class TuningOld {
//    // Tuners go here
//    @Tuner
//    public static Procedure mecanumTuner() {
//        return new MecanumTuner();
//    }
//
//
//    @Tuner
//    public static Procedure pinpointTuner() {
//        return new PinpointTuner();
//    }
//
//    @Tuner
//    public static Procedure foresightTuner() {
//        return new ForesightTuner((hardwareMap) -> new PinpointLocalizer(hardwareMap, ConstantsOld.localizerConfig), (hardwareMap) -> new Mecanum(hardwareMap, ConstantsOld.drivetrainConfig));
//    }
//
//    @Tuner
//    public static Procedure tests() {
//        return new Tests(hardwareMap -> new Mecanum(hardwareMap, ConstantsOld.drivetrainConfig), (hardwareMap -> new PinpointLocalizer(hardwareMap, ConstantsOld.localizerConfig)), () -> new Foresight(ConstantsOld.foresightConfig));
//    }
//
//
//}
