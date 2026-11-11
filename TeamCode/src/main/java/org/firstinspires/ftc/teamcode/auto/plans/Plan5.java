package org.firstinspires.ftc.teamcode.auto.plans;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.Config.Alliance;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.auto.Auto;

public class Plan5 {

    public static Pose start(Alliance alliance) {
        return new Auto.Field(alliance).startFront;
    }

    public static Path[] build(Alliance alliance) {
        Auto.Field f = new Auto.Field(alliance);
        PoseFactory p = f.poses;
        Pose frontHive = f.frontCell.pose;

        Path toPreloadShot = aim(line(f.startFront, f.shootFront), frontHive);
        return new Path[]{toPreloadShot};
    }

    public static long[] waitAfterMs(Alliance alliance) {
        return new long[]{shootMs()};
    }

    private static Path aim(Path path, Pose hive) {
        return path.heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(hive).reverse()));
    }

    private static long shootMs() {
        return Auto.AIM_MS + Math.max(1, Auto.GATE_OPENS_PER_SHOOT) * (Auto.SHOT_OPEN_MS + Auto.SHOT_GAP_MS) + Auto.SHOT_FINISH_MS;
    }
}
