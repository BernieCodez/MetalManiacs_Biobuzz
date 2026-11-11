package org.firstinspires.ftc.teamcode.auto.plans;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.Config.Alliance;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.auto.Auto;

public class Plan4 {

    public static Pose start(Alliance alliance) {
        Auto.Field f = new Auto.Field(alliance);
        PoseFactory p = f.poses;
        return p.of(85.3292, 5.1915, 90);
    }

    public static Path[] build(Alliance alliance) {
        Auto.Field f = new Auto.Field(alliance);
        PoseFactory p = f.poses;
        Pose frontHive = f.frontCell.pose;
        Pose rearHive = f.rearCell.pose;

        Path toPreloadShot = aim(line(p.of(85.3292, 5.1915, 90), p.of(85.087, 11.8986, 88.9126)), rearHive);
        Path toWallLane = face(line(p.of(85.087, 11.8986, 88.9126), p.of(132, 28, 83.3)), p.of(140, 96, 0));
        Path wallFlower = face(line(p.of(132, 28, 83.3), p.of(115.4, 96.5859, -4.9781)), p.of(140, 96, 0));
        Path wallFlowerShot = aim(line(p.of(115.4, 96.5859, -4.9781), f.wallFlowerShot), frontHive);
        Path reposition = face(line(f.wallFlowerShot, p.of(128, 134, -86.9)), p.of(135, 4, 0));
        return new Path[]{toPreloadShot, toWallLane, wallFlower, wallFlowerShot, reposition};
    }

    public static long[] waitAfterMs(Alliance alliance) {
        return new long[]{shootMs(), (alliance == Alliance.RED ? 2000 : 1750), Auto.FLOWER_INTAKE_MS, shootMs(), 0};
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
