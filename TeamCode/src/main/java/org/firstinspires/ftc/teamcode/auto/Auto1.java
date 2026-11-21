
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

    private final PoseFactory poseFactory =
            alliance == Alliance.BLUE
                    ? PoseFactory.degrees()
                    : PoseFactory.degrees().mirrorX(72).mirrorY(72);

    private final Pose start =
            poseFactory.of(87.289, 140.3444, 89.69);

    private final Pose shootOne =
            poseFactory.of(87, 139, 90);

    private final Pose shootOneSegment1Target =
            poseFactory.of(87, 87, 0);

    private final Pose pickUpPollen =
            poseFactory.of(137.3065, 141, 84.4535);

    private final Pose pickUpPollenControl1 =
            poseFactory.of(118.236, 112.3265, 0);

    private final Pose pickUpPollenSegment1Target =
            poseFactory.of(138, 144, 0);

    private final Pose shootTwo =
            poseFactory.of(85.5477, 9.9471, -91.7679);

    private final Pose shootTwoControl1 =
            poseFactory.of(109.8479, 47.5719, 0);

    private final Pose shootTwoSegment1Target =
            poseFactory.of(87, 57, 0);

    private final Pose pickUpFlowerPollen =
            poseFactory.of(94.3704, 10.6, -94.2004);

    private final Pose pickUpFlowerPollenSegment1Target =
            poseFactory.of(93.7, 4, 0);

    private final Pose shootThree =
            poseFactory.of(84.7934, 9.9083, -92.6828);

    private final Pose shootThreeSegment1Target =
            poseFactory.of(87, 57, 0);

    private final Pose pickUpFlowerPollen2 =
            poseFactory.of(126.5677, 94.0341, -0.0692);

    private final Pose pickUpFlowerPollen2Control1 =
            poseFactory.of(105.3375, 29.1419, 0);

    private final Pose pickUpFlowerPollen2Segment1Target =
            poseFactory.of(130, 94.03, 0);

    private final Pose park =
            poseFactory.of(133.2291, 45.8434, -87.5766);

    private final Pose parkSegment1Target =
            poseFactory.of(135, 4, 0);


    public Command autoRoutine() {
        return sequential(
                instant(() -> {
                    intakeTransferController.toggleForward();
                    intakeTransferController.flowerIntakeUp();
                    outtakeController.close();
                }),
                instant(() -> currentTargetHive = FRONT_CELL),

                waitMs(500),

                // MOVE 1 - Initial shooting position
                follow(follower, move1()),
                waitMs(1100),
                aimAndShoot(),

                // MOVE 2 - Garden pickup
                follow(follower, move2()),
                waitMs(0),

                instant(() -> currentTargetHive = REAR_CELL),

                // MOVE 4 - Second shooting position
                follow(follower, move4()),
                waitMs(1100),
                aimAndShoot(),

                // MOVE 5 - First flower intake
                instant(() -> intakeTransferController.flowerIntakeDown()),
                follow(follower, move5()),
                waitMs(1000),
                instant(() -> intakeTransferController.flowerIntakeUp()),

                instant(() -> currentTargetHive = REAR_CELL),

                // MOVE 6 - Third shooting position
                follow(follower, move6()),
                waitMs(1100),
                aimAndShoot(),

                // MOVE 7 - Second flower intake
                instant(() -> intakeTransferController.flowerIntakeDown()),
                follow(follower, move7()),
                waitMs(1000),
                instant(() -> intakeTransferController.flowerIntakeUp()),

                // PARK
                follow(follower, park()),
                waitMs(0)
        );
    }

    private Command aimAndShoot() {

        return sequential(

                instant(settleTimer::reset),

                waitUntil(() ->
                        follower.distanceToEndpoint() < SETTLE_DISTANCE_IN
                                || settleTimer.milliseconds() > SETTLE_TIMEOUT_MS
                ),

                instant(() -> shouldAutoAim = true),

                waitMs(AIM_MS),

                repeat(
                        sequential(
                                instant(shooter::openGate),
                                waitMs(AutoShooter.SHOT_OPEN_MS),
                                instant(shooter::closeGate),
                                waitMs(AutoShooter.SHOT_GAP_MS)
                        ),
                        Math.max(1, AutoShooter.GATE_OPENS_PER_SHOOT)
                ),

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

        intakeTransferController =
                new IntakeTransferController(hardwareMap);

        outtakeController =
                new OuttakeController(hardwareMap);

        shooter = new AutoShooter(outtakeController);

        autoAimController =
                new AutoAimController(hardwareMap);

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
                    ManualDrive.driveOrHold(
                            follower,
                            new DrivePowers(
                                    0,
                                    0,
                                    autoAimController.rotation
                            )
                    );
                }

                telemetry.addData("auto time", autoTimer.seconds());
                telemetry.addData("auto aim rotation", autoAimController.rotation);
                telemetry.addData("x", robot.x());
                telemetry.addData("y", robot.y());
                telemetry.addData("heading", robot.heading());

                if (follower.currentPath() != null) {
                    telemetry.addData(
                            "Current path distance remaining",
                            follower.distanceToEndpoint()
                    );

                    telemetry.addData(
                            "Path number",
                            follower.pathIndex()
                    );
                }

                telemetry.update();
            }

        } finally {
            OpModeStorage.autonomousEndPose = follower.pose();
        }
    }



    public Path move1() {

        return line(start, shootOne)
                .heading(
                        Interpolator.piecewise()
                                .until(
                                        1,
                                        Interpolator.facingPoint(
                                                shootOneSegment1Target
                                        ).reverse()
                                )
                );
    }


    public Path move2() {

        return curve(shootOne, pickUpPollenControl1, pickUpPollen)
                .heading(
                        Interpolator.piecewise()
                                .until(
                                        1,
                                        Interpolator.facingPoint(
                                                pickUpPollenSegment1Target
                                        )
                                )
                );
    }


    public Path move4() {

        return curve(pickUpPollen, shootTwoControl1, shootTwo)
                .heading(
                        Interpolator.piecewise()
                                .until(
                                        1,
                                        Interpolator.facingPoint(
                                                shootTwoSegment1Target
                                        ).reverse()
                                )
                );
    }


    public Path move5() {

        return line(shootTwo, pickUpFlowerPollen)
                .heading(
                        Interpolator.piecewise()
                                .until(
                                        1,
                                        Interpolator.facingPoint(
                                                pickUpFlowerPollenSegment1Target
                                        )
                                )
                );
    }


    public Path move6() {

        return line(pickUpFlowerPollen, shootThree)
                .heading(
                        Interpolator.piecewise()
                                .until(
                                        1,
                                        Interpolator.facingPoint(
                                                shootThreeSegment1Target
                                        ).reverse()
                                )
                );
    }


    public Path move7() {

        return curve(shootThree, pickUpFlowerPollen2Control1, pickUpFlowerPollen2)
                .heading(
                        Interpolator.piecewise()
                                .until(
                                        1,
                                        Interpolator.facingPoint(
                                                pickUpFlowerPollen2Segment1Target
                                        )
                                )
                );
    }


    public Path park() {

        return line(pickUpFlowerPollen2, park)
                .heading(
                        Interpolator.piecewise()
                                .until(
                                        1,
                                        Interpolator.facingPoint(
                                                parkSegment1Target
                                        )
                                )
                );
    }
}
