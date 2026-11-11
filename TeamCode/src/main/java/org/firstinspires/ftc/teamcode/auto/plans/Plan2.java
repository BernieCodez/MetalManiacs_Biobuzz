package org.firstinspires.ftc.teamcode.auto.plans;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static org.firstinspires.ftc.teamcode.Config.Alliance;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.auto.Auto;

public class Plan2 {

    public static Pose start(Alliance alliance) {
        return new Auto.Field(alliance).startWall;
    }

    public static Path[] build(Alliance alliance) {
        Auto.Field f = new Auto.Field(alliance);
        PoseFactory p = f.poses;

        Path leaveWall = tangent(line(f.startWall, p.of(133.4425, 72.7376, 179.4157)));

        return new Path[]{leaveWall};
    }

    public static long[] waitAfterMs(Alliance alliance) {
        return new long[]{0};
    }

    private static Path tangent(Path path) {
        return path.tangent();
    }
}
