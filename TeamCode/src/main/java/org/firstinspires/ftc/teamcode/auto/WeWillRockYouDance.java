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

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "WeWillRockYouDance", group = "Autonomous")
public class WeWillRockYouDance extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose rotateLeft = poseFactory.of(54.2555, 8, 100);
    private final Pose sweepPastMiddletoRight = poseFactory.of(57.4515, 8, 80);
    private final Pose returnCenter = poseFactory.of(56, 8, 90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, rotateLeft()),
                follow(follower, sweepPastMiddletoRight()),
                follow(follower, returnCenter())
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

    public Path rotateLeft() {
        return line(start, rotateLeft).linear(start, rotateLeft);
    }

    public Path sweepPastMiddletoRight() {
        return line(rotateLeft, sweepPastMiddletoRight).linear(rotateLeft, sweepPastMiddletoRight);
    }

    public Path returnCenter() {
        return line(sweepPastMiddletoRight, returnCenter).linear(sweepPastMiddletoRight, returnCenter);
    }
}
