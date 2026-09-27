package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.controllers.AutoAimController;
import org.firstinspires.ftc.teamcode.controllers.IntakeTransferController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "GauranshAuto", group = "Autonomous")
public class GauranshAuto extends LinearOpMode {

    private Follower follower;
    private IntakeTransferController intakeTransferController;
    private OuttakeController outtakeController;
    private AutoAimController autoAimController;
    private Config.Hive currentTargetHive = null;
    private boolean shouldAutoAim = false;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.6345, 10.5381, 90);
    private final Pose shoot = poseFactory.of(56.8632, 12.9903, 90);
    private final Pose garden = poseFactory.of(11.0448, 12.1659, -90);
    private final Pose gardenControl1 = poseFactory.of(32.8102, 38.5658, 0);
    private final Pose gardenSegment1Start = poseFactory.of(11.0448, 12.1659, 90);
    private final Pose gardenSegment1End = poseFactory.of(11.0448, 12.1659, -90);
    private final Pose shootAfterTip = poseFactory.of(58.7743, 118.932, -90);
    private final Pose shootAfterTipControl1 = poseFactory.of(8.5815, 77.4619, 0);
    private final Pose pollenIntake = poseFactory.of(47.2765, 127.784, -270);
    private final Pose pollenIntakeControl1 = poseFactory.of(49.7511, 113.0284, 0);
    private final Pose pollenIntakeSegment1Start = poseFactory.of(47.2765, 127.784, -90);
    private final Pose pollenIntakeSegment1End = poseFactory.of(47.2765, 127.784, 90);
    private final Pose shootAfterFlower = poseFactory.of(59.6129, 119.0329, 270);
    private final Pose shootAfterFlowerControl1 = poseFactory.of(49.6981, 112.6532, 0);
    private final Pose shootAfterFlowerSegment1Start = poseFactory.of(59.6129, 119.0329, 90);
    private final Pose shootAfterFlowerSegment1End = poseFactory.of(59.6129, 119.0329, -90);
    private final Pose park = poseFactory.of(14.6637, 116.1652, -90);
    private final Pose parkSegment1Start = poseFactory.of(14.6637, 116.1652, -90);
    private final Pose parkSegment1End = poseFactory.of(14.6637, 116.1652, -90);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                Commands.instant(() -> {
                    intakeTransferController.runIntake();
                    intakeTransferController.runGateOpen();
                }),
                parallel(
                        Commands.instant(() -> {
                            //currentTargetHive = Config.Hive.BLUE_TOP;
                            shouldAutoAim = true;
                        }),
                follow(follower, shoot())
                ),
                waitMs(2000),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = null;
                            shouldAutoAim = false;
                        }),
                follow(follower, garden())
                ),
                waitMs(3500),
                parallel(
                        Commands.instant(() -> {
                            //currentTargetHive = Config.Hive.BLUE_TOP;
                            shouldAutoAim = true;
                        }),
                follow(follower, shootaftertip())
                ),
                waitMs(2000),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = null;
                            shouldAutoAim = false;
                        }),
                follow(follower, pollenintake())
                ),
                waitMs(2500),
                parallel(
                        Commands.instant(() -> {
                            //currentTargetHive = Config.Hive.BLUE_TOP;
                            shouldAutoAim = true;
                        }),
                follow(follower, shootafterflower())
                ),
                waitMs(2000),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = null;
                            shouldAutoAim = false;
                        }),
                follow(follower, park())
                )
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

    public Path shoot() {
        return line(start, shoot).constant(shoot);
    }

    public Path garden() {
        return curve(shoot, gardenControl1, garden).heading(Interpolator.piecewise().until(1, Interpolator.linear(gardenSegment1Start, gardenSegment1End).reverse()));
    }

    public Path shootaftertip() {
        return curve(garden, shootAfterTipControl1, shootAfterTip).linear(garden, shootAfterTip);
    }

    public Path pollenintake() {
        return curve(shootAfterTip, pollenIntakeControl1, pollenIntake).heading(Interpolator.piecewise().until(1, Interpolator.linear(pollenIntakeSegment1Start, pollenIntakeSegment1End)));
    }

    public Path shootafterflower() {
        return curve(pollenIntake, shootAfterFlowerControl1, shootAfterFlower).heading(Interpolator.piecewise().until(1, Interpolator.linear(shootAfterFlowerSegment1Start, shootAfterFlowerSegment1End)));
    }

    public Path park() {
        return line(shootAfterFlower, park).heading(Interpolator.piecewise().until(1, Interpolator.linear(parkSegment1Start, parkSegment1End)));
    }
}