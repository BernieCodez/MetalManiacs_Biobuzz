package org.firstinspires.ftc.teamcode.auto.plans;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.Config.Alliance;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.auto.Auto;

public class Plan7 {

    public static Pose start(Alliance alliance) {
        return new Auto.Field(alliance).startFront;
    }

    public static Path[] build(Alliance alliance) {
        Auto.Field f = new Auto.Field(alliance);
        PoseFactory p = f.poses;
        Pose frontHive = f.frontCell.pose;
        Pose rearHive = f.rearCell.pose;

        Path toPreloadShot = aim(line(f.startFront, f.shootFront), frontHive);
        Path wallFlower = face(line(f.shootFront, f.wallFlower), f.wallFlowerLook);
        Path wallFlowerShot = aim(line(f.wallFlower, f.wallFlowerShot), frontHive);
        Path audienceFlower = face(curve(f.wallFlowerShot, new Pose(f.laneX, f.wallFlowerShot.y(), 0), new Pose(f.laneX, f.audienceFlower.y(), 0), f.audienceFlower), f.audienceFlowerLook);
        Path audienceFlowerShot = aim(line(f.audienceFlower, f.audienceFlowerShot), rearHive);
        return new Path[]{toPreloadShot, wallFlower, wallFlowerShot, audienceFlower, audienceFlowerShot};
    }

    public static long[] waitAfterMs(Alliance alliance) {
        return new long[]{shootMs(), Auto.FLOWER_INTAKE_MS, shootMs(), Auto.FLOWER_INTAKE_MS, shootMs()};
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
