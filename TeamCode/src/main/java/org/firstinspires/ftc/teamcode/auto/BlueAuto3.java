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

@Autonomous(name = "BlueAuto3", group = "Autonomous")
public class BlueAuto3 extends LinearOpMode {

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

    private final Alliance.Hive FRONT_CELL = Alliance.BLUE.TOP;
    private final Alliance.Hive REAR_CELL = Alliance.BLUE.BOTTOM;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(87.289, 140.3444, 270);
    private final Pose shootOne = poseFactory.of(87.1548, 133.3804, -91.4577);
    private final Pose pickUpPollen = poseFactory.of(134.5663, 134.272, 70.5585);
    private final Pose pickUpPollenControl1 = poseFactory.of(138.6976, 95.2488, 0);
    private final Pose pickUpPollenSegment1Target = poseFactory.of(138, 144, 0);
    private final Pose shootTwo = poseFactory.of(86.7787, 17.6124, 91.0525);
    private final Pose shootTwoControl1 = poseFactory.of(124, 75.991, 0);
    private final Pose pickUpFlowerPollen = poseFactory.of(96.4068, 14.3321, -86.7141);
    private final Pose pickUpFlowerPollenSegment1Target = poseFactory.of(97, 4, 0);
    private final Pose shootThree = poseFactory.of(86.5064, 133.4041, -90.639);
    private final Pose shootThreeControl1 = poseFactory.of(129.5497, 83.5041, 0);
    private final Pose parkAtEnd = poseFactory.of(133.6398, 29.0952, -65.6835);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> {
                    intakeTransferController.runIntake();
                    outtakeController.close();
                }),
                instant(() -> currentTargetHive = FRONT_CELL),
                follow(follower, shootOne()),
                aimAndShoot(4),
                follow(follower, pickUpPollen()),
                waitMs(1500),
                instant(() -> currentTargetHive = REAR_CELL),
                follow(follower, shootTwo()),
                aimAndShoot(4),
                follow(follower, pickUpFlowerPollen()),
                waitMs(1500),
                instant(() -> currentTargetHive = FRONT_CELL),
                follow(follower, shootThree()),
                aimAndShoot(4),
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
        return line(start, shootOne).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(FRONT_CELL.pose)));
    }

    public Path pickUpPollen() {
        return curve(shootOne, pickUpPollenControl1, pickUpPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpPollenSegment1Target)));
    }

    public Path shootTwo() {
        return curve(pickUpPollen, shootTwoControl1, shootTwo).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(REAR_CELL.pose)));
    }

    public Path pickUpFlowerPollen() {
        return line(shootTwo, pickUpFlowerPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpFlowerPollenSegment1Target)));
    }

    public Path shootThree() {
        return curve(pickUpFlowerPollen, shootThreeControl1, shootThree).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(FRONT_CELL.pose)));
    }

    public Path parkAtEnd() {
        return line(shootThree, parkAtEnd).tangent();
    }
}
