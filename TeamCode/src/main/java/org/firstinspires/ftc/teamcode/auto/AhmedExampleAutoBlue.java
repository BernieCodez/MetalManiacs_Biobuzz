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

    private final Pose start = poseFactory.of(85.5, 130.4213, 90);
    private final Pose pickuppollen = poseFactory.of(130.2978, 130.9775, 90.6112);
    private final Pose pickuppollenControl1 = poseFactory.of(131.1657, 70.8295, 0);
    private final Pose shoot = poseFactory.of(85.6337, 130.9652, -91.6303);
    private final Pose shootSegment1Target = poseFactory.of(83.5, 56, 0);
    private final Pose pickupmorepollen = poseFactory.of(132.4376, 131.2452, 90);
    private final Pose pickupmorepollenControl1 = poseFactory.of(134.5798, 69.6955, 0);
    private final Pose shoot_2 = poseFactory.of(85.3303, 130.8584, -91.4591);
    private final Pose shoot_2Segment1Target = poseFactory.of(83.5, 59, 0);
    private final Pose shoot_3 = poseFactory.of(83.9607, 11.2798, 91.2588);
    private final Pose shoot_3Control1 = poseFactory.of(138.4753, 67.5258, 0);
    private final Pose shoot_3Segment1Target = poseFactory.of(83, 55, 0);
    private final Pose park = poseFactory.of(132.2933, 26.7775, 17.7784);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, pickuppollen()),
                follow(follower, shoot()),
                follow(follower, pickupmorepollen()),
                follow(follower, shoot_2()),
                follow(follower, shoot_3()),
                follow(follower, park())
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

    public Path pickuppollen() {
        return curve(start, pickuppollenControl1, pickuppollen).tangent();
    }

    public Path shoot() {
        return line(pickuppollen, shoot).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shootSegment1Target)));
    }

    public Path pickupmorepollen() {
        return curve(shoot, pickupmorepollenControl1, pickupmorepollen).constant(pickupmorepollen);
    }

    public Path shoot_2() {
        return line(pickupmorepollen, shoot_2).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shoot_2Segment1Target)));
    }

    public Path shoot_3() {
        return curve(shoot_2, shoot_3Control1, shoot_3).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shoot_3Segment1Target)));
    }

    public Path park() {
        return line(shoot_3, park).tangent();
    }
}
