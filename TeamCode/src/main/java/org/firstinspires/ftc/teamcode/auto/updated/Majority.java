package org.firstinspires.ftc.teamcode.auto.updated;

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

import org.firstinspires.ftc.teamcode.auto.AutoShooter;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

import org.firstinspires.ftc.teamcode.controllers.IntakeTransferController;
import static org.firstinspires.ftc.teamcode.Config.*;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.controllers.AutoAimController;

@Autonomous(name = "Majority (Path 3)", group = "Autonomous")
public class Majority extends LinearOpMode {

    private static final long AIM_MS = 700;
    private static final long SHOT_FINISH_MS = 750;
    private static final double SETTLE_DISTANCE_IN = 1.5;
    private static final long SETTLE_TIMEOUT_MS = 1500;
    private static final double REAR_SHOT_FLYWHEEL_VELOCITY = 1115;
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
    private final Pose pickUpPollen = poseFactory.of(139.5348, 126.8552, 95.1153);
    private final Pose pickUpPollenControl1 = poseFactory.of(131.9595, 88.2177, 0);
    private final Pose pickUpPollenSegment1Target = poseFactory.of(138, 144, 0);
    private final Pose pickUpPollen2 = poseFactory.of(139.5347, 141.5, 93.5996);
    private final Pose pickUpPollen2Segment1Target = poseFactory.of(139, 150, 0);
    private final Pose shootTwo = poseFactory.of(80, 9.9471, -90);
    private final Pose shootTwoControl1 = poseFactory.of(90.8954, 112.0252, 0);
    private final Pose shootTwoControl2 = poseFactory.of(140.0766, 21.794, 0);
    private final Pose shootTwoSegment1Target = poseFactory.of(85, 57, 0);
    private final Pose approachFlowerPollen = poseFactory.of(96, 26.392, -90);
    private final Pose approachFlowerPollenSegment1Target = poseFactory.of(96, 18, 0);
    private final Pose pickUpFlowerPollen = poseFactory.of(95.3723, 4.584, -90);
    private final Pose pickUpFlowerPollenSegment1Target = poseFactory.of(95, 1, 0);
    private final Pose shootThree = poseFactory.of(87, 139, 90);
    private final Pose shootThreeControl1 = poseFactory.of(129.5497, 83.5041, 0);
    private final Pose shootThreeSegment1Target = poseFactory.of(87, 87, 0);
    private final Pose parkAtEnd = poseFactory.of(133.2291, 45.8434, 180);
    private final Pose parkAtEndSegment1Start = poseFactory.of(133.2291, 45.8434, 180);
    private final Pose parkAtEndSegment1End = poseFactory.of(133.2291, 45.8434, 180);

    public Command autoRoutine() {
        return sequential(
                instant(() -> currentTargetHive = FRONT_CELL),
                follow(follower, shootOne()),
                instant(() -> {
                    intakeTransferController.toggleForward();
                    intakeTransferController.flowerIntakeUp();
                    outtakeController.close();
                }),
                waitMs(3000),
                aimAndShoot(),
                follow(follower, pickUpPollen()),
                follow(follower, pickUpPollen2()),
                instant(() -> {
                    currentTargetHive = REAR_CELL;
                    shooter.setBaseVelocity(REAR_SHOT_FLYWHEEL_VELOCITY);
                }),
                follow(follower, shootTwo()),
                waitMs(300),
                aimAndShoot(),
                follow(follower, approachFlowerPollen()),
                instant(() -> intakeTransferController.flowerIntakeDown()),
                waitMs(100),
                follow(follower, pickUpFlowerPollen()),
                waitMs(900),
                instant(() -> intakeTransferController.flowerIntakeUp()),
                instant(() -> {
                    currentTargetHive = FRONT_CELL;
                    shooter.clearBaseVelocity();
                }),
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

    public Path pickUpPollen() {
        return curve(shootOne, pickUpPollenControl1, pickUpPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpPollenSegment1Target)));
    }

    public Path pickUpPollen2() {
        return line(pickUpPollen, pickUpPollen2).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpPollen2Segment1Target)));
    }

    public Path shootTwo() {
        return curve(pickUpPollen2, shootTwoControl1, shootTwoControl2, shootTwo).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shootTwoSegment1Target).reverse()));
    }

    public Path approachFlowerPollen() {
        return line(shootTwo, approachFlowerPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(approachFlowerPollenSegment1Target)));
    }

    public Path pickUpFlowerPollen() {
        return line(approachFlowerPollen, pickUpFlowerPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpFlowerPollenSegment1Target))).with(Constants.foresightConfig.maxPathSpeed.at(0.7));
    }

    public Path shootThree() {
        return curve(pickUpFlowerPollen, shootThreeControl1, shootThree).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shootThreeSegment1Target).reverse()));
    }

    public Path parkAtEnd() {
        return line(shootThree, parkAtEnd).heading(Interpolator.piecewise().until(1, Interpolator.linear(parkAtEndSegment1Start, parkAtEndSegment1End)));
    }
}
