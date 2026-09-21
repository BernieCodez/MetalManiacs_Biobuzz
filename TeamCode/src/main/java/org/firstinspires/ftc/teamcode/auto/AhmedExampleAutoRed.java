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

@Autonomous(name = "AhmedExampleAutoRed", group = "Autonomous")
public class AhmedExampleAutoRed extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.5899, 8.636, 90);
    private final Pose pickuppollen = poseFactory.of(8.9764, 8.5562, -95.9061);
    private final Pose pickuppollenControl1 = poseFactory.of(12.0404, 40.5483, 0);
    private final Pose shoot = poseFactory.of(57.1382, 8.8618, 88.9526);
    private final Pose shootSegment1Target = poseFactory.of(58, 56, 0);
    private final Pose pickupmorepollen = poseFactory.of(8.7444, 8.5059, 270);
    private final Pose pickupmorepollenControl1 = poseFactory.of(33.6303, 18.5011, 0);
    private final Pose shoot_2 = poseFactory.of(57.1236, 8.7551, 89.0007);
    private final Pose shoot_2Segment1Target = poseFactory.of(58, 59, 0);
    private final Pose shoot_3 = poseFactory.of(57.2213, 126.3876, -88.9476);
    private final Pose shoot_3Control1 = poseFactory.of(5.4551, 68.009, 0);
    private final Pose shoot_3Segment1Target = poseFactory.of(58, 84, 0);
    private final Pose park = poseFactory.of(7.2989, 112.6315, -164.5944);

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

        try{
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
