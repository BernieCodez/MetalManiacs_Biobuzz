package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "GauranshAuto", group = "Autonomous")
public class GauranshAuto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.6345, 10.5381, 90);
    private final Pose path1 = poseFactory.of(59.3842, 13.157, 0);
    private final Pose point2 = poseFactory.of(59.3842, 13.157, 90);
    private final Pose point2Segment1Heading = poseFactory.of(59.3842, 13.157, 90);
    private final Pose point3 = poseFactory.of(11.0448, 12.1659, -90);
    private final Pose point3Control1 = poseFactory.of(33.4447, 37.2967, 0);
    private final Pose point3Segment1Start = poseFactory.of(11.0448, 12.1659, 90);
    private final Pose point3Segment1End = poseFactory.of(11.0448, 12.1659, -90);
    private final Pose point4 = poseFactory.of(60.2549, 124.0082, -90);
    private final Pose point5 = poseFactory.of(47.2765, 131.3797, -270);
    private final Pose point5Control1 = poseFactory.of(49.7511, 113.0284, 0);
    private final Pose point5Segment1Start = poseFactory.of(47.2765, 131.3797, -90);
    private final Pose point5Segment1End = poseFactory.of(47.2765, 131.3797, 90);
    private final Pose point6 = poseFactory.of(59.6129, 124.5321, 270);
    private final Pose point6Control1 = poseFactory.of(49.6981, 112.6532, 0);
    private final Pose point6Segment1Start = poseFactory.of(59.6129, 124.5321, 90);
    private final Pose point6Segment1End = poseFactory.of(59.6129, 124.5321, -90);
    private final Pose point7 = poseFactory.of(57.1771, 12.3139, 0);
    private final Pose point7Control1 = poseFactory.of(12.6465, 59.3901, 0);
    private final Pose point7Segment1Start = poseFactory.of(57.1771, 12.3139, -90);
    private final Pose point7Segment1End = poseFactory.of(57.1771, 12.3139, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                waitMs(2000),
                follow(follower, path3()),
                follow(follower, path4()),
                waitMs(2000),
                follow(follower, path5()),
                waitMs(3500),
                follow(follower, path6()),
                waitMs(2000),
                follow(follower, path7())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

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
    }

    public Path path1() {
        return line(start, path1).constant(path1);
    }

    public Path path2() {
        return line(path1, point2).heading(Interpolator.piecewise().until(1, Interpolator.constant(point2Segment1Heading)));
    }

    public Path path3() {
        return curve(point2, point3Control1, point3).heading(Interpolator.piecewise().until(1, Interpolator.linear(point3Segment1Start, point3Segment1End).reverse()));
    }

    public Path path4() {
        return line(point3, point4).linear(point3, point4);
    }

    public Path path5() {
        return curve(point4, point5Control1, point5).heading(Interpolator.piecewise().until(1, Interpolator.linear(point5Segment1Start, point5Segment1End)));
    }

    public Path path6() {
        return curve(point5, point6Control1, point6).heading(Interpolator.piecewise().until(1, Interpolator.linear(point6Segment1Start, point6Segment1End)));
    }

    public Path path7() {
        return curve(point6, point7Control1, point7).heading(Interpolator.piecewise().until(1, Interpolator.linear(point7Segment1Start, point7Segment1End)));
    }
}
