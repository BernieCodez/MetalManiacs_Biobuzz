package org.firstinspires.ftc.teamcode.auto.plans;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.Config.Alliance;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.auto.Auto;

public class Plan1 {

    public static Pose start(Alliance alliance) {
        return new Auto.Field(alliance).startFront;
    }

    public static Path[] build(Alliance alliance) {
        Auto.Field f = new Auto.Field(alliance);
        PoseFactory p = f.poses;
        Pose frontHive = f.frontCell.pose;
        Pose rearHive = f.rearCell.pose;

        Path toPreloadShot = aim(line(f.startFront, f.shootFront), frontHive);
        Path toGarden = face(curve(f.shootFront, p.of(131.9595, 88.2177, 0), p.of(136.1938, 126.6587, 84.0537)), p.of(138, 144, 0));
        Path intoGarden = face(line(p.of(136.1938, 126.6587, 84.0537), p.of(136.1937, 139.1417, 90.0196)), p.of(136.19, 150, 0));
        Path toRearShot = aim(curve(p.of(136.1937, 139.1417, 90.0196), p.of(90.8954, 112.0252, 0), p.of(140.0766, 21.794, 0), p.of(84.435, 31.0886, 84.3466)), rearHive);
        Path audienceFlower = face(line(p.of(84.435, 31.0886, 84.3466), f.audienceFlower), f.audienceFlowerLook);
        Path audienceFlowerShot = aim(line(f.audienceFlower, f.audienceFlowerShot), rearHive);
        Path wallFlower = face(curve(f.audienceFlowerShot, p.of(114.0538, 25.2474, 0), f.wallFlower), f.wallFlowerLook);
        return new Path[]{toPreloadShot, toGarden, intoGarden, toRearShot, audienceFlower, audienceFlowerShot, wallFlower};
    }

    public static long[] waitAfterMs(Alliance alliance) {
        return new long[]{shootMs(), 0, 2000, shootMs() + 2000, Auto.FLOWER_INTAKE_MS, shootMs() + 2000, Auto.FLOWER_INTAKE_MS};
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
