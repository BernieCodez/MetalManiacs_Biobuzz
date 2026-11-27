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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@Autonomous(name = "Everything No Garden (Path 1) v7", group = "Autonomous")
public class EverythingNoGarden extends LinearOpMode {
    private double FRONT_FLYWHEEL_VELOCITY = 1070;
    private double BACK_FLYWHEEL_VELOCITY = 1070;
    ////////////////////////////////////////////////////////////////////////
    private static final long AIM_MS = 400;
    private static final long SHOT_FINISH_MS = 250;
    private static final double ALLOWED_SHOOTING_TIME = 5000; //2.5 seconds allowed to shoot
    private static final double SETTLE_DISTANCE_IN = 1.5;
    private static final long SETTLE_TIMEOUT_MS = 1500;
    private static final double RPM_THRESHOLD = 30;
    //    private static final double REAR_SHOT_FLYWHEEL_VELOCITY = 1081;
    private final ElapsedTime settleTimer = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();



    private Follower follower;

    private IntakeTransferController intakeTransfer;
    private OuttakeController outtake;
    private AutoAimController autoAim;
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
    private final Pose point2 = poseFactory.of(114, 95.1221, 0);
    private final Pose point2Segment1Target = poseFactory.of(140.15, 96, 0);
    private final Pose point3 = poseFactory.of(134, 95, 6.9952); //collect side flower
    private final Pose point3Segment1Target = poseFactory.of(140.15, 96, 0);
    private final Pose point4 = poseFactory.of(123.988, 95.0042, 0);
    private final Pose point3_2 = poseFactory.of(130.0976, 94.6899, 0);
    private final Pose shootTwo = poseFactory.of(96, 10, -85);                //SHOT 2
    private final Pose shootTwoControl1 = poseFactory.of(114.9576, 28.3121, 0);
    private final Pose shootTwoSegment1Target = poseFactory.of(84, 57, 0);
    private final Pose approachFlowerPollen = poseFactory.of(96, 9.9, -90); //BACK FLOWER!
    private final Pose approachFlowerPollenSegment1Target = poseFactory.of(96, 18, 0);
    private final Pose pickUpFlowerPollen = poseFactory.of(95.3723, 6, -90); //COLLET BACK FLOWER | was 4
    private final Pose pickUpFlowerPollenSegment1Target = poseFactory.of(95, 1, 0);
    private final Pose point9 = poseFactory.of(95.415, 9.7571, -90);
    private final Pose point10 = poseFactory.of(95.6542, 5.1037, -90);
    private final Pose shootThree = poseFactory.of(96, 10, -90);                 //SHOT 3!
    private final Pose shootThreeControl1 = poseFactory.of(90.5256, 14.8378, 0);
    private final Pose shootThreeSegment1Target = poseFactory.of(84, 57, 0);
    private final Pose parkAtEnd = poseFactory.of(133.2291, 45.8434, 180);
    private final Pose parkAtEndSegment1Start = poseFactory.of(133.2291, 45.8434, 180);
    private final Pose parkAtEndSegment1End = poseFactory.of(133.2291, 45.8434, 180);

    public Command autoRoutine() {
        return sequential(
                //outtake should start up here
                instant(() -> {
                    currentTargetHive = FRONT_CELL;
                    outtake.targetVelocity = FRONT_FLYWHEEL_VELOCITY;
                }),
                follow(follower, shootOne()),//move forward to shooting position
                instant(() -> {
                    intakeTransfer.toggleForward();
                    intakeTransfer.flowerIntakeUp();
                }),

                instant(timer::reset),
                waitUntil(() -> outtake.flywheel.getVelocity() > outtake.getTargetVelocity()-RPM_THRESHOLD || timer.milliseconds() > 3000), //wait till flywheel done starting up or it takes more than 3 seconds
                instant(() ->{
                    telemetry.addData("Flywheel Velocity", outtake.flywheel.getVelocity());
                    telemetry.addData("Timer",timer);
                    telemetry.update();
                }),
                aimAndShoot(),
                parallel( //lower intake as it goes to face the flower
                        instant(() -> intakeTransfer.flowerIntakeDown()),
                        follow(follower, faceFlower())
                ),
//                waitMs(200),
                follow(follower, pushFlower()),
                waitMs(1500),
                follow(follower, backOffFlower()),
//                follow(follower, pushFlowerAgain()),
                instant(() -> {
                    currentTargetHive = REAR_CELL;
                    outtake.targetVelocity = BACK_FLYWHEEL_VELOCITY;
                }), //for auto aim
                parallel( //raise intake while moving
                        sequential(
                                instant(() -> intakeTransfer.flowerIntakeUp()),
                                waitMs(1500),
                                instant(() -> intakeTransfer.flowerIntakeDown())
                        ),
                        follow(follower, shootTwo())
                ),
                aimAndShoot(),

                follow(follower, approachFlowerPollen()),
//                waitMs(200),
                follow(follower, pickUpFlowerPollen()),
                waitMs(1500),
                follow(follower, backOffFlowerPollen()),
//                follow(follower, pickUpFlowerPollenAgain()),
                instant(() -> currentTargetHive = REAR_CELL),
                follow(follower, shootThree()),
                waitMs(250),
                aimAndShoot(),
                parallel(
                        instant(() -> intakeTransfer.flowerIntakeUp()),
                        follow(follower, parkAtEnd())
                )
        );
    }

    private Command aimAndShoot() {
        return sequential(
                instant(timer::reset),
                waitUntil(() -> follower.distanceToEndpoint() < SETTLE_DISTANCE_IN || timer.milliseconds() > SETTLE_TIMEOUT_MS), //make sure robot is near the end of its path
                instant(() -> shouldAutoAim = true),
                waitMs(AIM_MS),
                instant(timer::reset),
                instant(() -> outtake.startBurst()),
                waitUntil(() -> outtake.shotPhase == OuttakeController.ShotPhase.IDLE || timer.milliseconds() > ALLOWED_SHOOTING_TIME),
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
        intakeTransfer = new IntakeTransferController(hardwareMap);
        outtake = new OuttakeController(hardwareMap);
        autoAim = new AutoAimController(hardwareMap);
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
                if (currentTargetHive != null) { //dont need auto aim
                    autoAim.update(currentTargetHive, robot);
                }
                outtake.update();
                intakeTransfer.update();

                if (shouldAutoAim) {
                    ManualDrive.driveOrHold(follower, new DrivePowers(0, 0, autoAim.rotation));
                }

//                telemetry.addData("auto time", autoTimer.seconds());
//                telemetry.addData("auto aim rotation", autoAimController.rotation);
//                telemetry.addData("x", robot.x());
//                telemetry.addData("y", robot.y());
//                telemetry.addData("heading", robot.heading());




//                telemetry.addData("right flywheel speed", outtake.flywheel.getRightVelocity());
//                telemetry.addData("left flywheel speed", outtake.flywheel.getLeftVelocity());
//                telemetry.addData("target velocity",outtake.getTargetVelocity());
//
//
//                if (follower.currentPath() != null) {
//                    telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
//                    telemetry.addData("Path number", follower.pathIndex());
//                }
//
//                telemetry.update();
            }

        } finally {
            OpModeStorage.autonomousEndPose = follower.pose();
        }
    }

    public Path shootOne() {
        return line(start, shootOne).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shootOneSegment1Target).reverse()));
    }

    public Path faceFlower() {
        return line(shootOne, point2).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(point2Segment1Target)));
    }

    public Path pushFlower() {
        return line(point2, point3).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(point3Segment1Target))).with(Constants.foresightConfig.maxPathSpeed.at(0.7));
    }

    public Path backOffFlower() {
        return line(point3, point4).reverseTangent();
    }

    public Path pushFlowerAgain() {
        return line(point4, point3_2).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(point3Segment1Target))).with(Constants.foresightConfig.maxPathSpeed.at(0.7));
    }

    public Path shootTwo() {
        return curve(point3_2, shootTwoControl1, shootTwo).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shootTwoSegment1Target).reverse()));
    }

    public Path approachFlowerPollen() {
//        return line(shootTwo, approachFlowerPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(approachFlowerPollenSegment1Target)));
        return line(shootTwo, approachFlowerPollen).linear(shootTwo, approachFlowerPollen);
    }

    public Path pickUpFlowerPollen() {
        return line(approachFlowerPollen, pickUpFlowerPollen).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(pickUpFlowerPollenSegment1Target))).with(Constants.foresightConfig.maxPathSpeed.at(0.7));
    }

    public Path backOffFlowerPollen() {
        return line(pickUpFlowerPollen, point9).reverseTangent();
    }

    public Path pickUpFlowerPollenAgain() {
        return line(point9, point10).tangent().with(Constants.foresightConfig.maxPathSpeed.at(0.7));
    }

    public Path shootThree() {
        return curve(point10, shootThreeControl1, shootThree).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(shootThreeSegment1Target).reverse()));
    }

    public Path parkAtEnd() {
        return line(shootThree, parkAtEnd).heading(Interpolator.piecewise().until(1, Interpolator.linear(parkAtEndSegment1Start, parkAtEndSegment1End)));
    }
}
