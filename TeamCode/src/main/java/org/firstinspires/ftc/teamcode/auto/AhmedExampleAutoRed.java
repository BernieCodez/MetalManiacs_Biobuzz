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

@Autonomous(name = "AhmedExampleAutoRed", group = "Autonomous")
public class AhmedExampleAutoRed extends LinearOpMode {

    private Follower follower;

    private IntakeTransferController intakeTransferController;
    private OuttakeController outtakeController;
    private AutoAimController autoAimController;
    private Alliance.Hive currentTargetHive = null;
    private boolean shouldAutoAim = false;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.711, 3.6556, 90);
    private final Pose shootOne = poseFactory.of(56.8452, 10.6196, 88.5424);
    private final Pose shootOneSegment1Target = poseFactory.of(58, 56, 0);
    private final Pose pickUpPollen = poseFactory.of(9.4337, 9.728, -109.4413);
    private final Pose pickUpPollenControl1 = poseFactory.of(5.3024, 48.7512, 0);
    private final Pose pickUpPollenSegment1Target = poseFactory.of(6, 0, 0);
    private final Pose shootTwo = poseFactory.of(57.2213, 126.3876, -88.9476);
    private final Pose shootTwoControl1 = poseFactory.of(5.4551, 68.009, 0);
    private final Pose shootTwoSegment1Target = poseFactory.of(58, 84, 0);
    private final Pose pickUpFlowerPollen = poseFactory.of(47.5932, 129.6679, 93.2861);
    private final Pose pickUpFlowerPollenSegment1Target = poseFactory.of(47, 140, 0);
    private final Pose shootThree = poseFactory.of(57.4936, 10.5959, 90.3801);
    private final Pose shootThreeControl1 = poseFactory.of(14.4503, 60.4959, 0);
    private final Pose shootThreeSegment1Target = poseFactory.of(57, 85, 0);
    private final Pose parkAtEnd = poseFactory.of(10.3602, 114.9048, 114.3165);
    private final Pose returnTestingOnly = poseFactory.of(56.9037, 3.3996, 89.9025);
    private final Pose returnTestingOnlySegment1Target = poseFactory.of(57, 60, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                Commands.instant(() -> {
                    intakeTransferController.runIntake();
                    intakeTransferController.runGateOpen();
                }),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = Alliance.RED.TOP;
                            shouldAutoAim = true;
                        }),
                        follow(follower, shootOne())
                ),
                waitMs(1000),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = null;
                            shouldAutoAim = false;
                        }),
                        follow(follower, pickUpPollen())
                ),
                waitMs(1500),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = Alliance.RED.BOTTOM;
                            shouldAutoAim = true;
                        }),
                        follow(follower, shootTwo())
                ),
                waitMs(1000),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = null;
                            shouldAutoAim = false;
                        }),
                        follow(follower, pickUpFlowerPollen())
                ),
                waitMs(1500),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = Alliance.RED.TOP;
                            shouldAutoAim = true;
                        }),
                        follow(follower, shootThree())
                ),
                waitMs(1000),
                parallel(
                        Commands.instant(() -> {
                            currentTargetHive = null;
                            shouldAutoAim = false;
                        }),
                        follow(follower, parkAtEnd())
                ),
                waitMs(10000),
                parallel(
                        follow(follower, returnTestingOnly())
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

    public Path shootOne() {
        return line(start, shootOne).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shootOneSegment1Target)));
    }

    public Path pickUpPollen() {
        return curve(shootOne, pickUpPollenControl1, pickUpPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpPollenSegment1Target)));
    }

    public Path shootTwo() {
        return curve(pickUpPollen, shootTwoControl1, shootTwo).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shootTwoSegment1Target)));
    }

    public Path pickUpFlowerPollen() {
        return line(shootTwo, pickUpFlowerPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpFlowerPollenSegment1Target)));
    }

    public Path shootThree() {
        return curve(pickUpFlowerPollen, shootThreeControl1, shootThree).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shootThreeSegment1Target)));
    }

    public Path parkAtEnd() {
        return line(shootThree, parkAtEnd).tangent();
    }

    public Path returnTestingOnly() {
        return line(parkAtEnd, returnTestingOnly).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(returnTestingOnlySegment1Target)));
    }
}