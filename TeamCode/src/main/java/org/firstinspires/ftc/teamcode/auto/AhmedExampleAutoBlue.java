package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
//©JavaDude
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

@Autonomous(name = "AhmedExampleAutoBlue", group = "Autonomous")
public class AhmedExampleAutoBlue extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(83.4493, 138.6242, -90);
    private final Pose shoot = poseFactory.of(83.8759, 131.8441, -90.284);
    private final Pose shootSegment1Target = poseFactory.of(83.5, 56, 0);
    private final Pose pickuppollen = poseFactory.of(133.2947, 131.1577, 87.3621);
    private final Pose pickuppollenControl1 = poseFactory.of(131.4844, 86.4455, 0);
    private final Pose shoot_2 = poseFactory.of(84.891, 14.8982, 89.8443);
    private final Pose shoot_2Control1 = poseFactory.of(124.8255, 39.7027, 0);
    private final Pose shoot_2Segment1Target = poseFactory.of(85, 55, 0);
    private final Pose pickupflowerpollen = poseFactory.of(94.9546, 14.8057, -94.623);
    private final Pose pickupflowerpollenSegment1Target = poseFactory.of(94, 3, 0);
    private final Pose shoot_3 = poseFactory.of(85.072, 15.1745, 91.5419);
    private final Pose shoot_3Segment1Target = poseFactory.of(84, 55, 0);
    private final Pose park = poseFactory.of(131.3923, 25.6532, 12.747);
    private final Pose point7 = poseFactory.of(83.6398, 137.9669, -89.6377);
    private final Pose point7Segment1Target = poseFactory.of(84, 81, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, shoot()),
                waitMs(1000),
                follow(follower, pickuppollen()),
                waitMs(1500),
                follow(follower, shoot_2()),
                waitMs(1000),
                follow(follower, pickupflowerpollen()),
                waitMs(1500),
                follow(follower, shoot_3()),
                waitMs(1000),
                follow(follower, park()),
                waitMs(10000),
                follow(follower, path7())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        try {
            waitForStart();
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();

                telemetry.addData("x", follower.pose().x());
                telemetry.addData("y", follower.pose().y());
                telemetry.addData("heading", follower.pose().heading());

                if (follower.currentPath() != null) {
                    telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                    telemetry.addData("Path number", follower.pathIndex());
                }

                telemetry.update();
            }

        } finally {
            //robots stopping position carries over to teleop!
            if (follower != null) {
                OpModeStorage.autonomousEndPose = follower.pose();
            }
        }
    }

    public Path shoot() {
        return line(start, shoot).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shootSegment1Target)));
    }

    public Path pickuppollen() {
        return curve(shoot, pickuppollenControl1, pickuppollen).tangent();
    }

    public Path shoot_2() {
        return curve(pickuppollen, shoot_2Control1, shoot_2).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shoot_2Segment1Target)));
    }

    public Path pickupflowerpollen() {
        return line(shoot_2, pickupflowerpollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickupflowerpollenSegment1Target)));
    }

    public Path shoot_3() {
        return line(pickupflowerpollen, shoot_3).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shoot_3Segment1Target)));
    }

    public Path park() {
        return line(shoot_3, park).tangent();
    }

    public Path path7() {
        return line(park, point7).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(point7Segment1Target)));
    }
}