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

@Autonomous(name = "Everything (Path 1) v34", group = "Autonomous")
public class Everything extends LinearOpMode {
    private double FRONT_FLYWHEEL_VELOCITY = 1070;
    private double BACK_FLYWHEEL_VELOCITY = 1070;
    private double lurchForward=0;
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

    //Garden Poses
    private final Pose faceGarden = poseFactory.of(141, 128, 90);
    private final Pose faceGardenControl1 = poseFactory.of(113.3363, 123.0206, 0);
    private final Pose pickUpGarden = poseFactory.of(141, 136, 90);

    private final Pose point2 = poseFactory.of(114, 95.1221, 0);
    private final Pose point2Segment1Target = poseFactory.of(140.15, 96, 0);
    private final Pose point3 = poseFactory.of(134, 95, 6.9952); //collect side flower
    private final Pose point3Segment1Target = poseFactory.of(140.15, 96, 0);
    private final Pose point4 = poseFactory.of(123.988, 95.0042, 0);
    private final Pose point3_2 = poseFactory.of(130.0976, 94.6899, 0);
    private final Pose shootTwo = poseFactory.of(96, 11, -75.3791);             //SHOT 2
    private final Pose shootTwoControl1 = poseFactory.of(115, 23, 0);
    private final Pose shootTwoSegment1Heading = poseFactory.of(96, 11, 90);
    private final Pose shootTwoSegment2Target = poseFactory.of(84, 57, 0);
    private final Pose approachFlowerPollen = poseFactory.of(96, 9.9, -90); //BACK FLOWER!
    private final Pose approachFlowerPollenSegment1Target = poseFactory.of(96, 18, 0);
    private final Pose pickUpFlowerPollen = poseFactory.of(95.3723, 6, -90); //COLLET BACK FLOWER | was 4
    private final Pose pickUpFlowerPollenSegment1Target = poseFactory.of(95, 1, 0);
    private final Pose point9 = poseFactory.of(95.415, 9.7571, -90);
    private final Pose point10 = poseFactory.of(95.6542, 5.1037, -90);
    private final Pose shootThree = poseFactory.of(96, 11, -90);                 //SHOT 3!
    private final Pose shootThreeControl1 = poseFactory.of(90.5256, 14.8378, 0);
    private final Pose shootThreeSegment1Target = poseFactory.of(84, 57, 0);

    //second flower
    private final Pose faceFlower = poseFactory.of(130, 95, 9.2356);
    private final Pose faceFlowerControl1 = poseFactory.of(132.0375, 18.7276, 0);
    private final Pose faceFlowerSegment1Target = poseFactory.of(140.15, 96, 0);

    private final Pose parkAtEnd = poseFactory.of(133.1037, 15, -35);
    private final Pose parkAtEndControl1 = poseFactory.of(105.043, 10.4676, 0);
    private final Pose parkAtEndSegment1Target = poseFactory.of(140.15, 96, 0);
    private final Pose parkAtEndSegment2Start = poseFactory.of(133.1037, 20, 57.9477);
    private final Pose parkAtEndSegment2End = poseFactory.of(133.1037, 20, 325);

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

                //Calling Garden Methods
                parallel(
                        follow(follower, faceGarden()),
                        instant(() -> intakeTransfer.flowerIntakeDown())
                ),
                follow(follower, pickUpGarden()),
                follow(follower, backupGarden()),
                waitMs(250),
                follow(follower, pickUpAgainGarden()),
                waitMs(500),

                instant(() -> {
                    currentTargetHive = REAR_CELL;
                    outtake.targetVelocity = BACK_FLYWHEEL_VELOCITY;
                }), //for auto aim

                follow(follower, shootTwo()),
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
                //                  OLD CODE FOR FLOWER INTAKE vvv
                parallel( //lower intake as it goes to face the flower
                        sequential(
                                instant(() -> intakeTransfer.flowerIntakeUp()),
                                waitMs(1500),
                                instant(() -> intakeTransfer.flowerIntakeDown())
                            ),
                        follow(follower, faceFlower())
                ),
//                waitMs(200),
                follow(follower, pushFlower()),
                waitMs(1500),
                follow(follower, backOffFlower()),
//                follow(follower, pushFlowerAgain()),
                parallel(
                        instant(() -> intakeTransfer.flowerIntakeUp()),
                        follow(follower, parkAtEnd())
                )
        );
    }

    private Command aimAndShoot() {
//REPOSITION LURCH
        return sequential(
                instant(timer::reset),
                waitUntil(() -> follower.distanceToEndpoint() < SETTLE_DISTANCE_IN
                        || timer.milliseconds() > SETTLE_TIMEOUT_MS),

                instant(() -> {
                    shouldAutoAim = true;
                }),
                waitMs(AIM_MS),

                instant(() -> {
                    timer.reset();
                    outtake.startBurst();
                    lurchForward = 0.15;
                }),

                waitMs(300),

                // Lurch forward
                instant(() -> lurchForward = -0.50),
                waitMs(500),

                // Lurch backward
                instant(() -> lurchForward = 1),
                waitMs(150),

                instant(() -> lurchForward = 0),

                waitUntil(() -> outtake.shotPhase == OuttakeController.ShotPhase.IDLE
                        || timer.milliseconds() > ALLOWED_SHOOTING_TIME),

                waitMs(SHOT_FINISH_MS),

                instant(() -> {
                    lurchForward = 0;
                    shouldAutoAim = false;
                    currentTargetHive = null;
                })
        );
//FAST ONE TIME LURCH
//        return sequential(
//                instant(timer::reset),
//                waitUntil(() -> follower.distanceToEndpoint() < SETTLE_DISTANCE_IN
//                        || timer.milliseconds() > SETTLE_TIMEOUT_MS),
//
//                instant(() -> {
//                    shouldAutoAim = true;
//                    lurchForward = 0.15;
//                }),
//                waitMs(AIM_MS),
//
//                instant(() -> {
//                    timer.reset();
//                    outtake.startBurst();
//                    lurchForward = 0;
//                }),
//
//                waitMs(500),
//
//                // Lurch forward
//                instant(() -> lurchForward = -0.75),
//                waitMs(350),
//
//                // Lurch backward
//                instant(() -> lurchForward = 1),
//                waitMs(150),
//
//                instant(() -> lurchForward = 0),
//
//                waitUntil(() -> outtake.shotPhase == OuttakeController.ShotPhase.IDLE
//                        || timer.milliseconds() > ALLOWED_SHOOTING_TIME),
//
//                waitMs(SHOT_FINISH_MS),
//
//                instant(() -> {
//                    lurchForward = 0;
//                    shouldAutoAim = false;
//                    currentTargetHive = null;
//                })
//        );

//BACK AND FORTH LURCHING
//        return sequential(
//                instant(timer::reset),
//                waitUntil(() -> follower.distanceToEndpoint() < SETTLE_DISTANCE_IN
//                        || timer.milliseconds() > SETTLE_TIMEOUT_MS),
//
//                instant(() -> {
//                    shouldAutoAim = true;
//                    lurchForward = 0.15;
//                }),
//                waitMs(AIM_MS),
//
//                instant(() -> {
//                    timer.reset();
//                    outtake.startBurst();
////                    lurchForward = 0.15;
//                    lurchForward = 0;
//                }),
//
//                waitMs(500),
//
//                // Lurch forward
//                instant(() -> lurchForward = -1),
//                waitMs(150),
//
//                // Lurch backward
//                instant(() -> lurchForward = 1),
//                waitMs(100),
//
//                // Lurch forward again for final ball
//                instant(() -> lurchForward = -1),
//                waitMs(150),
//
//                // Lurch backward
//                instant(() -> lurchForward = 1),
//                waitMs(250),
//
//                instant(() -> lurchForward = 0),
//
//                waitUntil(() -> outtake.shotPhase == OuttakeController.ShotPhase.IDLE
//                        || timer.milliseconds() > ALLOWED_SHOOTING_TIME),
//
//                waitMs(SHOT_FINISH_MS),
//
//                instant(() -> {
//                    lurchForward = 0;
//                    shouldAutoAim = false;
//                    currentTargetHive = null;
//                })
//        );
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
                    ManualDrive.driveOrHold(follower, new DrivePowers(lurchForward, 0, autoAim.rotation));
                }

//                telemetry.addData("auto time", autoTimer.seconds());
//                telemetry.addData("auto aim rotation", autoAimController.rotation);
                telemetry.addData("x", robot.x());
                telemetry.addData("y", robot.y());
//                telemetry.addData("heading", robot.heading());
                telemetry.update();



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


    //Garden Methods
    public Path faceGarden() {
        return curve(start, faceGardenControl1, faceGarden).linear(start, faceGarden);
    }

    public Path pickUpGarden() {
        return line(faceGarden, pickUpGarden).constant(pickUpGarden);
    }

    public Path backupGarden() {
        return line(pickUpGarden, faceGarden).constant(pickUpGarden);
    }

    public Path pickUpAgainGarden(){
        return line(faceGarden, pickUpGarden).constant(pickUpGarden);
    }

    public Path faceFlower() {
        return curve(shootThree, faceFlowerControl1, faceFlower).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(faceFlowerSegment1Target)));
    }

    public Path pushFlower() {
        return line(faceFlower, point3).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(point3Segment1Target))).with(Constants.foresightConfig.maxPathSpeed.at(0.7));
    }

    public Path backOffFlower() {
        return line(point3, point4).reverseTangent();
    }

    public Path pushFlowerAgain() {
        return line(point4, point3_2).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(point3Segment1Target))).with(Constants.foresightConfig.maxPathSpeed.at(0.7));
    }

    public Path shootTwo() {
        return curve(pickUpGarden, shootTwoControl1, shootTwo).heading(Interpolator.piecewise().until(0.3022, Interpolator.constant(shootTwoSegment1Heading)).until(1, Interpolator.facingPoint(shootTwoSegment2Target).reverse()));
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
        return curve(point4, parkAtEndControl1, parkAtEnd).heading(Interpolator.piecewise().until(0.2194, Interpolator.facingPoint(parkAtEndSegment1Target)).until(1, Interpolator.linear(parkAtEndSegment2Start, parkAtEndSegment2End)));
    }
}
