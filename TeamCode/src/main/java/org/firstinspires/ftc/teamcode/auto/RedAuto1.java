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

@Autonomous(name = "RedAuto1", group = "Autonomous")
public class RedAuto1 extends LinearOpMode {

    private static final long AIM_MS = 700;
    private static final long SHOT_OPEN_MS = 250;
    private static final long SHOT_GAP_MS = 250;
    private static final long SHOT_FINISH_MS = 750;

    private Follower follower;

    private IntakeTransferController intakeTransferController;
    private OuttakeController outtakeController;
    private AutoAimController autoAimController;
    private Alliance.Hive currentTargetHive = null;
    private boolean shouldAutoAim = false;

    private final Alliance.Hive FRONT_CELL = Alliance.RED.TOP;
    private final Alliance.Hive REAR_CELL = Alliance.RED.BOTTOM;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.711, 3.6556, 90);
    private final Pose shootOne = poseFactory.of(56.8452, 10.6196, 88.5423);
    private final Pose pickUpPollen = poseFactory.of(8.8478, 17.9309, -99.0243);
    private final Pose pickUpPollenControl1 = poseFactory.of(12.0405, 55.7823, 0);
    private final Pose pickUpPollenSegment1Target = poseFactory.of(6, 0, 0);
    private final Pose pickUpPollen2 = poseFactory.of(8.9451, 3.9355, -89.6836);
    private final Pose pickUpPollen2Segment1Target = poseFactory.of(9, -6, 0);
    private final Pose shootTwo = poseFactory.of(59.565, 112.9114, -93.0984);
    private final Pose shootTwoControl1 = poseFactory.of(53.1046, 31.9748, 0);
    private final Pose shootTwoControl2 = poseFactory.of(3.9234, 122.206, 0);
    private final Pose pickUpFlowerPollen = poseFactory.of(49.9369, 131.4257, 87.575);
    private final Pose pickUpFlowerPollenSegment1Target = poseFactory.of(50.3, 140, 0);
    private final Pose shootThree = poseFactory.of(59.763, 113.1356, -93.4627);
    private final Pose pickUpFlowerPollen2 = poseFactory.of(12.6077, 45.3406, -174.1213);
    private final Pose pickUpFlowerPollen2Control1 = poseFactory.of(29.9462, 118.7526, 0);
    private final Pose pickUpFlowerPollen2Segment1Target = poseFactory.of(9.3, 45, 0);
    private final Pose parkAtEnd = poseFactory.of(12.411, 122.5217, 90.146);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> {
                    intakeTransferController.setPower(1);
                    outtakeController.close();
                }),
                instant(() -> currentTargetHive = FRONT_CELL),
                follow(follower, shootOne()),
                aimAndShoot(4),
                follow(follower, pickUpPollen()),
                follow(follower, pickUpPollen2()),
                waitMs(1500),
                instant(() -> currentTargetHive = REAR_CELL),
                follow(follower, shootTwo()),
                aimAndShoot(4),
                follow(follower, pickUpFlowerPollen()),
                waitMs(1500),
                instant(() -> currentTargetHive = REAR_CELL),
                follow(follower, shootThree()),
                aimAndShoot(4),
                follow(follower, pickUpFlowerPollen2()),
                waitMs(1500),
                follow(follower, parkAtEnd())
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

    public Path shootOne() {
        return line(start, shootOne).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(FRONT_CELL.pose).reverse()));
    }

    public Path pickUpPollen() {
        return curve(shootOne, pickUpPollenControl1, pickUpPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpPollenSegment1Target)));
    }

    public Path pickUpPollen2() {
        return line(pickUpPollen, pickUpPollen2).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpPollen2Segment1Target)));
    }

    public Path shootTwo() {
        return curve(pickUpPollen2, shootTwoControl1, shootTwoControl2, shootTwo).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(REAR_CELL.pose).reverse()));
    }

    public Path pickUpFlowerPollen() {
        return line(shootTwo, pickUpFlowerPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpFlowerPollenSegment1Target)));
    }

    public Path shootThree() {
        return line(pickUpFlowerPollen, shootThree).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(REAR_CELL.pose).reverse()));
    }

    public Path pickUpFlowerPollen2() {
        return curve(shootThree, pickUpFlowerPollen2Control1, pickUpFlowerPollen2).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpFlowerPollen2Segment1Target)));
    }

    public Path parkAtEnd() {
        return line(pickUpFlowerPollen2, parkAtEnd).tangent();
    }
}
