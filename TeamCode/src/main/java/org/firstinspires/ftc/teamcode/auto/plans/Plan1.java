package org.firstinspires.ftc.teamcode.auto.plans;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

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

@Autonomous(name = "Plan1", group = "Autonomous")
public class Plan1 extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(87.289, 140.3444, 89.69);
    private final Pose move1 = poseFactory.of(87, 139, 90);
    private final Pose move1Segment1Target = poseFactory.of(87, 87, 0);
    private final Pose move2 = poseFactory.of(136.1938, 126.6587, 84.0537);
    private final Pose move2Control1 = poseFactory.of(131.9595, 88.2177, 0);
    private final Pose move2Segment1Target = poseFactory.of(138, 144, 0);
    private final Pose move3 = poseFactory.of(136.1937, 139.1417, 90.0195);
    private final Pose move3Segment1Target = poseFactory.of(136.19, 150, 0);
    private final Pose move4 = poseFactory.of(84.435, 31.0886, -95.6534);
    private final Pose move4Control1 = poseFactory.of(90.8954, 112.0252, 0);
    private final Pose move4Control2 = poseFactory.of(140.0766, 21.794, 0);
    private final Pose move4Segment1Target = poseFactory.of(87, 57, 0);
    private final Pose move5 = poseFactory.of(94.8492, 22.4006, -93.5738);
    private final Pose move5Segment1Target = poseFactory.of(93.7, 4, 0);
    private final Pose move6 = poseFactory.of(84.4335, 31.0609, -95.6506);
    private final Pose move6Segment1Target = poseFactory.of(87, 57, 0);
    private final Pose move7 = poseFactory.of(124.8986, 98.8559, 0.8423);
    private final Pose move7Control1 = poseFactory.of(114.0538, 25.2474, 0);
    private final Pose move7Segment1Target = poseFactory.of(134.7, 99, 0);
    private final Pose park = poseFactory.of(133.6, 26, -86.3588);
    private final Pose parkSegment1Target = poseFactory.of(135, 4, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, move1()),
                follow(follower, move2()),
                follow(follower, move3()),
                follow(follower, move4()),
                follow(follower, move5()),
                follow(follower, move6()),
                follow(follower, move7()),
                follow(follower, park())
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

    public Path move1() {
        return Paths.line(start, move1).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(move1Segment1Target).reverse()));
    }

    public Path move2() {
        return Paths.curve(move1, move2Control1, move2).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(move2Segment1Target)));
    }

    public Path move3() {
        return Paths.line(move2, move3).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(move3Segment1Target)));
    }

    public Path move4() {
        return Paths.curve(move3, move4Control1, move4Control2, move4).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(move4Segment1Target).reverse()));
    }

    public Path move5() {
        return Paths.line(move4, move5).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(move5Segment1Target)));
    }

    public Path move6() {
        return Paths.line(move5, move6).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(move6Segment1Target).reverse()));
    }

    public Path move7() {
        return Paths.curve(move6, move7Control1, move7).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(move7Segment1Target)));
    }

    public Path park() {
        return Paths.line(move7, park).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(parkSegment1Target)));
    }
}
