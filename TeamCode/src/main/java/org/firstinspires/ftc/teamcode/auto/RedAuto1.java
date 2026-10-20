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

    private final Pose start = poseFactory.of(87.289, 140.3444, 270);
    private final Pose shootOne = poseFactory.of(87.1548, 133.3804, -91.4577);
    private final Pose pickUpPollen = poseFactory.of(135.1522, 126.0691, 80.9757);
    private final Pose pickUpPollenControl1 = poseFactory.of(131.9595, 88.2177, 0);
    private final Pose pickUpPollenSegment1Target = poseFactory.of(138, 144, 0);
    private final Pose pickUpPollen2 = poseFactory.of(135.0549, 140.0645, 90.3164);
    private final Pose pickUpPollen2Segment1Target = poseFactory.of(135, 150, 0);
    private final Pose shootTwo = poseFactory.of(84.435, 31.0886, 86.9016);
    private final Pose shootTwoControl1 = poseFactory.of(90.8954, 112.0252, 0);
    private final Pose shootTwoControl2 = poseFactory.of(140.0766, 21.794, 0);
    private final Pose pickUpFlowerPollen = poseFactory.of(94.0631, 12.5743, -92.425);
    private final Pose pickUpFlowerPollenSegment1Target = poseFactory.of(93.7, 4, 0);
    private final Pose shootThree = poseFactory.of(84.237, 30.8644, 86.5373);
    private final Pose pickUpFlowerPollen2 = poseFactory.of(131.3923, 98.6594, 5.8787);
    private final Pose pickUpFlowerPollen2Control1 = poseFactory.of(114.0538, 25.2474, 0);
    private final Pose pickUpFlowerPollen2Segment1Target = poseFactory.of(134.7, 99, 0);
    private final Pose parkAtEnd = poseFactory.of(131.589, 21.4783, -89.854);

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
        return line(start, shootOne).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(FRONT_CELL.pose)));
    }

    public Path pickUpPollen() {
        return curve(shootOne, pickUpPollenControl1, pickUpPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpPollenSegment1Target)));
    }

    public Path pickUpPollen2() {
        return line(pickUpPollen, pickUpPollen2).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpPollen2Segment1Target)));
    }

    public Path shootTwo() {
        return curve(pickUpPollen2, shootTwoControl1, shootTwoControl2, shootTwo).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(REAR_CELL.pose)));
    }

    public Path pickUpFlowerPollen() {
        return line(shootTwo, pickUpFlowerPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpFlowerPollenSegment1Target)));
    }

    public Path shootThree() {
        return line(pickUpFlowerPollen, shootThree).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(REAR_CELL.pose)));
    }

    public Path pickUpFlowerPollen2() {
        return curve(shootThree, pickUpFlowerPollen2Control1, pickUpFlowerPollen2).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpFlowerPollen2Segment1Target)));
    }

    public Path parkAtEnd() {
        return line(pickUpFlowerPollen2, parkAtEnd).tangent();
    }
}
