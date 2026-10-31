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

@Autonomous(name = "Auto4", group = "Autonomous")
public class Auto4 extends LinearOpMode {

    private static final long AIM_MS = 700;
    private static final long SHOT_OPEN_MS = 250;
    private static final long SHOT_GAP_MS = 250;
    private static final long SHOT_FINISH_MS = 750;
    private static final double SETTLE_DISTANCE_IN = 1.5;
    private static final long SETTLE_TIMEOUT_MS = 1500;
    private final ElapsedTime settleTimer = new ElapsedTime();

    private static final long WAIT_BEFORE_PRELOAD_MS = 3250;
    private final long WAIT_BEFORE_WALL_FLOWER_MS = currentAlliance == Alliance.RED ? 2000 : 1750;
    private static final long WAIT_BEFORE_PARK_MS = 0;

    private Follower follower;

    private IntakeTransferController intakeTransferController;
    private OuttakeController outtakeController;
    private AutoAimController autoAimController;
    private Alliance.Hive currentTargetHive = null;
    private boolean shouldAutoAim = false;

    private final Alliance alliance = currentAlliance;

    private final Alliance.Hive FRONT_CELL = alliance.TOP;
    private final Alliance.Hive REAR_CELL = alliance.BOTTOM;

    private final PoseFactory poseFactory = alliance == Alliance.BLUE
            ? PoseFactory.degrees()
            : PoseFactory.degrees().mirrorX(72).mirrorY(72);

    private final Pose start = poseFactory.of(85.3292, 5.1915, 90);
    private final Pose waitThenShootPreloadsBackCell = poseFactory.of(85.087, 11.8986, 88.9126);
    private final Pose holdForPartner = poseFactory.of(132, 28, 83.3);
    private final Pose waitForUsThenWallFlower = poseFactory.of(115.4, 96.5859, -4.9781);
    private final Pose waitForUsThenWallFlowerSegment1Target = poseFactory.of(140, 96, 0);
    private final Pose shootFrontCell = poseFactory.of(86.3095, 133.7184, -90.3879);
    private final Pose clearFrontCell = poseFactory.of(128, 134, -86.9);
    private final Pose waitForUsThenPark = poseFactory.of(133.6, 26, -86.4);
    private final Pose waitForUsThenParkSegment1Target = poseFactory.of(135, 4, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> {
                    intakeTransferController.toggleForward();
                    intakeTransferController.flowerIntakeUp();
                    outtakeController.close();
                }),
                waitMs(WAIT_BEFORE_PRELOAD_MS),
                instant(() -> currentTargetHive = REAR_CELL),
                follow(follower, waitThenShootPreloadsBackCell()),
                aimAndShoot(4),
                follow(follower, holdForPartner()),
                waitMs(WAIT_BEFORE_WALL_FLOWER_MS),
                instant(() -> intakeTransferController.flowerIntakeDown()),
                follow(follower, waitForUsThenWallFlower()),
                waitMs(1500),
                instant(() -> intakeTransferController.flowerIntakeUp()),
                instant(() -> currentTargetHive = FRONT_CELL),
                follow(follower, shootFrontCell()),
                aimAndShoot(4),
                follow(follower, clearFrontCell()),
                waitMs(WAIT_BEFORE_PARK_MS),
                follow(follower, waitForUsThenPark())
        );
    }

    private Command aimAndShoot(int pollen) {
        return sequential(
                instant(settleTimer::reset),
                waitUntil(() -> follower.distanceToEndpoint() < SETTLE_DISTANCE_IN || settleTimer.milliseconds() > SETTLE_TIMEOUT_MS),
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
                outtakeController.update(currentTargetHive != null ? currentTargetHive : FRONT_CELL, robot);
                intakeTransferController.update();

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
        return line(start, waitThenShootPreloadsBackCell).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(REAR_CELL.pose).reverse()));
    }

    public Path holdForPartner() {
        return line(waitThenShootPreloadsBackCell, holdForPartner).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(waitForUsThenWallFlowerSegment1Target)));
    }

    public Path waitForUsThenWallFlower() {
        return line(holdForPartner, waitForUsThenWallFlower).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(waitForUsThenWallFlowerSegment1Target)));
    }

    public Path shootFrontCell() {
        return line(waitForUsThenWallFlower, shootFrontCell).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(FRONT_CELL.pose).reverse()));
    }

    public Path clearFrontCell() {
        return line(shootFrontCell, clearFrontCell).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(waitForUsThenParkSegment1Target)));
    }

    public Path waitForUsThenPark() {
        return line(clearFrontCell, waitForUsThenPark).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(waitForUsThenParkSegment1Target)));
    }
}
