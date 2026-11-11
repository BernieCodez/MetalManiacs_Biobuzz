package org.firstinspires.ftc.teamcode.auto.plans;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.Config.Alliance;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.auto.Auto;

public class Plan3 {

    public static Pose start(Alliance alliance) {
        Auto.Field f = new Auto.Field(alliance);
        PoseFactory p = f.poses;
        return p.of(87.289, 140.3444, -90);
    }

    public static Path[] build(Alliance alliance) {
        Auto.Field f = new Auto.Field(alliance);
        PoseFactory p = f.poses;
        Pose frontHive = f.frontCell.pose;
        Pose rearHive = f.rearCell.pose;

        Path toPreloadShot = aim(line(p.of(87.289, 140.3444, -90), p.of(87.1548, 133.3804, -91.4576)), frontHive);
        Path toGarden = face(curve(p.of(87.1548, 133.3804, -91.4576), p.of(131.9595, 88.2177, 0), p.of(139.5348, 126.8552, 95.1153)), p.of(138, 144, 0));
        Path intoGarden = face(line(p.of(139.5348, 126.8552, 95.1153), p.of(139.5347, 141.5, 93.5996)), p.of(139, 150, 0));
        Path toRearShot = aim(curve(p.of(139.5347, 141.5, 93.5996), p.of(90.8954, 112.0252, 0), p.of(140.0766, 21.794, 0), p.of(84.435, 31.0886, -95.6534)), rearHive);
        Path audienceFlower = face(line(p.of(84.435, 31.0886, -95.6534), p.of(95.0457, 30.4583, -92.9117)), f.audienceFlowerLook);
        Path toFrontShot = aim(curve(p.of(95.0457, 30.4583, -92.9117), p.of(129.5497, 83.5041, 0), p.of(86.5064, 133.4041, -90.639)), frontHive);
        return new Path[]{toPreloadShot, toGarden, intoGarden, toRearShot, audienceFlower, toFrontShot};
    }

    public static long[] waitAfterMs(Alliance alliance) {
        return new long[]{shootMs(), 0, 2000, shootMs(), Auto.FLOWER_INTAKE_MS, shootMs()};
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
