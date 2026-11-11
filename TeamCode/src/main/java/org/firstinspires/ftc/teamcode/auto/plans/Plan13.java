package org.firstinspires.ftc.teamcode.auto.plans;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.Config.Alliance;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.auto.Auto;

public class Plan13 {

    public static Pose start(Alliance alliance) {
        return new Auto.Field(alliance).startFront;
    }

    public static Path[] build(Alliance alliance) {
        Auto.Field f = new Auto.Field(alliance);
        PoseFactory p = f.poses;
        Pose frontHive = f.frontCell.pose;

        Path toPreloadShot = aim(line(f.startFront, f.shootFront), frontHive);
        Path toGarden = face(line(f.shootFront, f.garden), f.gardenLook);
        Path intoGarden = face(line(f.garden, f.garden2), f.garden2Look);
        Path gardenShot = aim(line(f.garden2, f.gardenShot), frontHive);
        return new Path[]{toPreloadShot, toGarden, intoGarden, gardenShot};
    }

    public static long[] waitAfterMs(Alliance alliance) {
        return new long[]{shootMs(), 0, Auto.GARDEN_INTAKE_MS, shootMs()};
    }

    private static Path aim(Path path, Pose hive) {
        return path.heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(hive).reverse()));
    }

    private static Path face(Path path, Pose point) {
        return path.heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(point)));
    }

    private static long shootMs() {
        return Auto.AIM_MS + Math.max(1, Auto.GATE_OPENS_PER_SHOOT) * (Auto.SHOT_OPEN_MS + Auto.SHOT_GAP_MS) + Auto.SHOT_FINISH_MS;
    }
}
