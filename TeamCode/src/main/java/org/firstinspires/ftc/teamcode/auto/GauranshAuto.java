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

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1Start = poseFactory.of(56, 8, 0);
    private final Pose path1 = poseFactory.of(59.3842, 13.157, 90);
    private final Pose point2 = poseFactory.of(8.7182, 8.9933, -90);
    private final Pose point2Control1 = poseFactory.of(28.58, 15.9342, 0);
    private final Pose point2Segment1Start = poseFactory.of(8.7182, 8.9933, 90);
    private final Pose point2Segment1End = poseFactory.of(8.7182, 8.9933, -90);
    private final Pose point3 = poseFactory.of(57.9283, 126.3348, -90);
    private final Pose point4 = poseFactory.of(58.3569, 126.5206, -90);
    private final Pose point4Segment1Start = poseFactory.of(58.3569, 126.5206, -90);
    private final Pose point4Segment1End = poseFactory.of(58.3569, 126.5206, -90);
    private final Pose point5 = poseFactory.of(47.2765, 131.3797, -270);
    private final Pose point5Control1 = poseFactory.of(49.7511, 113.0284, 0);
    private final Pose point5Segment1Start = poseFactory.of(47.2765, 131.3797, -90);
    private final Pose point5Segment1End = poseFactory.of(47.2765, 131.3797, 90);
    private final Pose point6 = poseFactory.of(58.3438, 126.6472, 270);
    private final Pose point6Control1 = poseFactory.of(49.6981, 112.6532, 0);
    private final Pose point6Segment1Start = poseFactory.of(58.3438, 126.6472, 90);
    private final Pose point6Segment1End = poseFactory.of(58.3438, 126.6472, -90);
    private final Pose point7 = poseFactory.of(56.5426, 10.8333, 131.7671);
    private final Pose point7Control1 = poseFactory.of(12.6465, 59.3901, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
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
        return line(path1Start, path1).linear(path1Start, path1);
    }

    public Path path2() {
        return curve(path1, point2Control1, point2).heading(Interpolator.piecewise().until(1, Interpolator.linear(point2Segment1Start, point2Segment1End).reverse()));
    }

    public Path path3() {
        return line(point2, point3).linear(point2, point3);
    }

    public Path path4() {
        return line(point3, point4).heading(Interpolator.piecewise().until(1, Interpolator.linear(point4Segment1Start, point4Segment1End)));
    }

    public Path path5() {
        return curve(point4, point5Control1, point5).heading(Interpolator.piecewise().until(1, Interpolator.linear(point5Segment1Start, point5Segment1End)));
    }

    public Path path6() {
        return curve(point5, point6Control1, point6).heading(Interpolator.piecewise().until(1, Interpolator.linear(point6Segment1Start, point6Segment1End)));
    }

    public Path path7() {
        return curve(point6, point7Control1, point7).reverseTangent();
    }
}
