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
import static com.pedropathing.ivy.commands.Commands.*;
//©JavaDude
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

import org.firstinspires.ftc.teamcode.controllers.IntakeTransferController;
import static org.firstinspires.ftc.teamcode.Config.*;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.controllers.AutoAimController;

@Autonomous(name = "AhmedExampleAutoBlue", group = "Autonomous")
public class AhmedExampleAutoBlue extends LinearOpMode {

    private Follower follower;

    private IntakeTransferController intakeTransferController;
    private OuttakeController outtakeController;
    private AutoAimController autoAimController;
    private Alliance.Hive currentTargetHive = null;
    private boolean shouldAutoAim = false;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.711, 3.6556, 90);
    private final Pose shoot = poseFactory.of(56.8452, 10.6196, 88.5424);
    private final Pose shootSegment1Target = poseFactory.of(58, 56, 0);
    private final Pose pickuppollen = poseFactory.of(9.4337, 9.728, -109.4413);
    private final Pose pickuppollenControl1 = poseFactory.of(5.3024, 48.7512, 0);
    private final Pose pickuppollenSegment1Target = poseFactory.of(6, 0, 0);
    private final Pose shoot_2 = poseFactory.of(57.2213, 126.3876, -88.9476);
    private final Pose shoot_2Control1 = poseFactory.of(5.4551, 68.009, 0);
    private final Pose shoot_2Segment1Target = poseFactory.of(58, 84, 0);
    private final Pose pickupflowerpollen = poseFactory.of(47.5932, 129.6679, 93.2861);
    private final Pose pickupflowerpollenSegment1Target = poseFactory.of(47, 140, 0);
    private final Pose shoot_3 = poseFactory.of(57.4936, 126.3154, -90.6845);
    private final Pose shoot_3Segment1Target = poseFactory.of(57, 85, 0);
    private final Pose park = poseFactory.of(12.9969, 115.7836, -166.684);
    private final Pose returntestingonly = poseFactory.of(56.9037, 3.3996, 89.9025);
    private final Pose returntestingonlySegment1Target = poseFactory.of(57, 60, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                Commands.instant(() -> {
                    intakeTransferController.runIntake();
                    intakeTransferController.runGateOpen();
                }),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = Alliance.BLUE.TOP;
                            shouldAutoAim = true;
                        }),
                        follow(follower, shoot())
                ),
                waitMs(1000),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = null;
                            shouldAutoAim = false;
                        }),
                        follow(follower, pickuppollen())
                ),
                waitMs(1500),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = Alliance.BLUE.TOP;
                            shouldAutoAim = true;
                        }),
                        follow(follower, shoot_2())
                ),
                waitMs(1000),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = null;
                            shouldAutoAim = false;
                        }),
                        follow(follower, pickupflowerpollen())
                ),
                waitMs(1500),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = Alliance.BLUE.TOP;
                            shouldAutoAim = true;
                        }),
                        follow(follower, shoot_3())
                ),
                waitMs(1000),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = null;
                            shouldAutoAim = false;
                        }),
                        follow(follower, park())
                ),
                waitMs(10000),
                parallel(
                        follow(follower, returntestingonly())
                )
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        intakeTransferController = new IntakeTransferController(hardwareMap);
        outtakeController = new OuttakeController(hardwareMap);
        autoAimController = new AutoAimController(hardwareMap);
        follower.setPose(start);
        follower.update();

        try {
            waitForStart();
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();

                outtakeController.update(currentTargetHive, follower.pose());
                autoAimController.update(currentTargetHive, follower.pose(), shouldAutoAim, 0.0);

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
        return curve(shoot, pickuppollenControl1, pickuppollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickuppollenSegment1Target)));
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

    public Path returntestingonly() {
        return line(park, returntestingonly).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(returntestingonlySegment1Target)));
    }
}