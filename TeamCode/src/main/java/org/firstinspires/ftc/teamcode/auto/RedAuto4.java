package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.paths.interpolator.Interpolator;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

import org.firstinspires.ftc.teamcode.controllers.IntakeTransferController;
import static org.firstinspires.ftc.teamcode.Config.*;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.controllers.AutoAimController;

@Autonomous(name = "RedAuto4", group = "Autonomous")
public class RedAuto4 extends LinearOpMode {

    private static final long AIM_MS = 700;
    private static final long SHOT_OPEN_MS = 250;
    private static final long SHOT_GAP_MS = 250;
    private static final long SHOT_FINISH_MS = 750;

    private static final long WAIT_BEFORE_PRELOAD_MS = 3000;
    private static final long WAIT_BEFORE_WALL_FLOWER_MS = 2000;
    private static final long WAIT_BEFORE_PARK_MS = 1000;

    private Follower follower;

    private IntakeTransferController intakeTransferController;
    private OuttakeController outtakeController;
    private AutoAimController autoAimController;
    private Alliance.Hive currentTargetHive = null;
    private boolean shouldAutoAim = false;

    private final Alliance.Hive FRONT_CELL = Alliance.RED.TOP;
    private final Alliance.Hive REAR_CELL = Alliance.RED.BOTTOM;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.6708, 138.8085, 270);
    private final Pose waitThenShootPreloadsBackCell = poseFactory.of(58.913, 132.1014, -91.0874);
    private final Pose waitForUsThenWallFlower = poseFactory.of(10.7267, 47.4141, 175.0219);
    private final Pose waitForUsThenWallFlowerSegment1Target = poseFactory.of(4, 48, 0);
    private final Pose shootFrontCell = poseFactory.of(57.6905, 10.2816, 89.6121);
    private final Pose waitForUsThenPark = poseFactory.of(9.5859, 96.6874, 90.775);
    private final Pose waitForUsThenParkSegment1Target = poseFactory.of(9, 140, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> {
                    intakeTransferController.runIntake();
                    outtakeController.close();
                }),
                waitMs(WAIT_BEFORE_PRELOAD_MS),
                instant(() -> currentTargetHive = REAR_CELL),
                follow(follower, waitThenShootPreloadsBackCell()),
                aimAndShoot(4),
                waitMs(WAIT_BEFORE_WALL_FLOWER_MS),
                follow(follower, waitForUsThenWallFlower()),
                waitMs(1500),
                instant(() -> currentTargetHive = FRONT_CELL),
                follow(follower, shootFrontCell()),
                aimAndShoot(4),
                waitMs(WAIT_BEFORE_PARK_MS),
                follow(follower, waitForUsThenPark())
        );
    }

    private Command aimAndShoot(int pollen) {
        return sequential(
                instant(() -> shouldAutoAim = true),
                waitMs(AIM_MS),
                repeat(sequential(
                        instant(() -> outtakeController.fire()),
                        waitMs(SHOT_OPEN_MS),
                        instant(() -> outtakeController.close()),
                        waitMs(SHOT_GAP_MS)
                ), pollen),
                waitMs(SHOT_FINISH_MS),
                instant(() -> {
                    shouldAutoAim = false;
                    currentTargetHive = null;
                })
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

        ElapsedTime autoTimer = new ElapsedTime();

        try {
            waitForStart();
            autoTimer.reset();
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();

                Pose robot = follower.pose();
                if (currentTargetHive != null) {
                    autoAimController.update(currentTargetHive, robot);
                }
                outtakeController.update(currentTargetHive, robot);

                if (shouldAutoAim) {
                    ManualDrive.driveOrHold(follower, new DrivePowers(0, 0, autoAimController.rotation));
                }

                telemetry.addData("auto time", autoTimer.seconds());
                telemetry.addData("auto aim rotation", autoAimController.rotation);
                telemetry.addData("x", robot.x());
                telemetry.addData("y", robot.y());
                telemetry.addData("heading", robot.heading());

                if (follower.currentPath() != null) {
                    telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                    telemetry.addData("Path number", follower.pathIndex());
                }

                telemetry.update();
            }

        } finally {
            //robots stopping position carries over to teleop!
            OpModeStorage.autonomousEndPose = follower.pose();
        }
    }

    public Path waitThenShootPreloadsBackCell() {
        return line(start, waitThenShootPreloadsBackCell).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(REAR_CELL.pose)));
    }

    public Path waitForUsThenWallFlower() {
        return line(waitThenShootPreloadsBackCell, waitForUsThenWallFlower).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(waitForUsThenWallFlowerSegment1Target)));
    }

    public Path shootFrontCell() {
        return line(waitForUsThenWallFlower, shootFrontCell).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(FRONT_CELL.pose)));
    }

    public Path waitForUsThenPark() {
        return line(shootFrontCell, waitForUsThenPark).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(waitForUsThenParkSegment1Target)));
    }
}
