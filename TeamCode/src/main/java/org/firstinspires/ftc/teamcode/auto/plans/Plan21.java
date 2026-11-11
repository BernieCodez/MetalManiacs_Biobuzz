package org.firstinspires.ftc.teamcode.auto.plans;

import static org.firstinspires.ftc.teamcode.Config.Alliance;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.auto.Auto;

public class Plan21 {

    public static Pose start(Alliance alliance, int side) {
        return new Auto.Field(alliance).startPose(side);
    }

    public static Path[] build(Alliance alliance, int side) {
        return new Path[]{};
    }

    public static long[] waitAfterMs(Alliance alliance) {
        return new long[]{};
    }
}
