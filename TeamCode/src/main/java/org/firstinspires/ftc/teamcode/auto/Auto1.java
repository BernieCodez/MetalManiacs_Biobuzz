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

@Autonomous(name = "Auto1", group = "Autonomous")
public class Auto1 extends LinearOpMode {

    private static final long AIM_MS = 700;
    private static final long SHOT_FINISH_MS = 750;
    private static final double SETTLE_DISTANCE_IN = 1.5;
    private static final long SETTLE_TIMEOUT_MS = 1500;
    private static final double REAR_SHOT_FLYWHEEL_VELOCITY = 1150;
    private final ElapsedTime settleTimer = new ElapsedTime();

    private Follower follower;

    private IntakeTransferController intakeTransferController;
    private OuttakeController outtakeController;
    private AutoShooter shooter;
    private AutoAimController autoAimController;
    private Alliance.Hive currentTargetHive = null;
    private boolean shouldAutoAim = false;

    private final Alliance alliance = currentAlliance;

    private final Alliance.Hive FRONT_CELL = alliance.TOP;
    private final Alliance.Hive REAR_CELL = alliance.BOTTOM;

    private final PoseFactory poseFactory = alliance == Alliance.BLUE
            ? PoseFactory.degrees()
            : PoseFactory.degrees().mirrorX(72).mirrorY(72);

    private final Pose start = poseFactory.of(87.289, 140.3444, 89.69);
    private final Pose shootOne = poseFactory.of(87, 139, 90);
    private final Pose shootOneSegment1Target = poseFactory.of(87, 87, 0);
    private final Pose pickUpGarden = poseFactory.of(141, 133.52, 90.2204);
    private final Pose pickUpGardenControl1 = poseFactory.of(141.5, 110.5469, 0);
    private final Pose pickUpGardenSegment1Target = poseFactory.of(141.5, 0, 0);
    private final Pose shootTwo = poseFactory.of(81, 9.9471, -97.2669);
    private final Pose shootTwoControl1 = poseFactory.of(114.9576, 28.3121, 0);
    private final Pose shootTwoSegment1Target = poseFactory.of(87, 57, 0);
    private final Pose approachFlowerPollen = poseFactory.of(96, 26.392, -90);
    private final Pose approachFlowerPollenSegment1Target = poseFactory.of(96, 18, 0);
    private final Pose pickUpFlowerPollen = poseFactory.of(95.3723, 4.584, -95.9305);
    private final Pose pickUpFlowerPollenSegment1Target = poseFactory.of(95, 1, 0);
    private final Pose shootThree = poseFactory.of(81, 9.9471, -97.2669);
    private final Pose shootThreeControl1 = poseFactory.of(90.5256, 14.8378, 0);
    private final Pose shootThreeSegment1Target = poseFactory.of(87, 57, 0);
    private final Pose parkAtEnd = poseFactory.of(133.2291, 45.8434, 0);
    private final Pose parkAtEndSegment1Start = poseFactory.of(133.2291, 45.8434, 0);
    private final Pose parkAtEndSegment1End = poseFactory.of(133.2291, 45.8434, 0);

    public Command autoRoutine() {
        return sequential(
                instant(() -> {
                    intakeTransferController.toggleForward();
                    intakeTransferController.flowerIntakeUp();
                    outtakeController.close();
                }),
                instant(() -> currentTargetHive = FRONT_CELL),
                waitMs(4000),
                follow(follower, shootOne()),
                waitMs(300),
                aimAndShoot(),
                instant(() -> intakeTransferController.flowerIntakeDown()),
                follow(follower, pickUpGarden()),
                waitMs(850),
                instant(() -> intakeTransferController.flowerIntakeUp()),
                instant(() -> {
                    currentTargetHive = REAR_CELL;
                    shooter.setBaseVelocity(REAR_SHOT_FLYWHEEL_VELOCITY);
                }),
                follow(follower, shootTwo()),
                waitMs(300),
                aimAndShoot(),
                follow(follower, approachFlowerPollen()),
                instant(() -> intakeTransferController.flowerIntakeDown()),
                waitMs(500),
                follow(follower, pickUpFlowerPollen()),
                waitMs(870),
                instant(() -> intakeTransferController.flowerIntakeUp()),
                instant(() -> currentTargetHive = REAR_CELL),
                follow(follower, shootThree()),
                waitMs(300),
                aimAndShoot(),
                follow(follower, parkAtEnd())
        );
    }

    private Command aimAndShoot() {
        return sequential(
                instant(settleTimer::reset),
                waitUntil(() -> follower.distanceToEndpoint() < SETTLE_DISTANCE_IN || settleTimer.milliseconds() > SETTLE_TIMEOUT_MS),
                instant(() -> shouldAutoAim = true),
                waitMs(AIM_MS),
                repeat(sequential(
                        instant(shooter::openGate),
                        waitMs(AutoShooter.SHOT_OPEN_MS),
                        instant(shooter::closeGate),
                        waitMs(AutoShooter.SHOT_GAP_MS)
                ), Math.max(1, AutoShooter.GATE_OPENS_PER_SHOOT)),
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
        shooter = new AutoShooter(outtakeController);
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
                shooter.update();
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
            OpModeStorage.autonomousEndPose = follower.pose();
        }
    }

    public Path shootOne() {
        return line(start, shootOne).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shootOneSegment1Target).reverse()));
    }

    public Path pickUpGarden() {
        return curve(shootOne, pickUpGardenControl1, pickUpGarden).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpGardenSegment1Target).reverse())).with(Constants.foresightConfig.maxPathSpeed.at(0.5));
    }

    public Path shootTwo() {
        return curve(pickUpGarden, shootTwoControl1, shootTwo).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shootTwoSegment1Target).reverse()));
    }

    public Path approachFlowerPollen() {
        return line(shootTwo, approachFlowerPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(approachFlowerPollenSegment1Target)));
    }

    public Path pickUpFlowerPollen() {
        return line(approachFlowerPollen, pickUpFlowerPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpFlowerPollenSegment1Target)));
    }

    public Path shootThree() {
        return curve(pickUpFlowerPollen, shootThreeControl1, shootThree).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shootThreeSegment1Target).reverse()));
    }

    public Path parkAtEnd() {
        return line(shootThree, parkAtEnd).heading(Interpolator.piecewise().until(1, Interpolator.linear(parkAtEndSegment1Start, parkAtEndSegment1End)));
    }
}
