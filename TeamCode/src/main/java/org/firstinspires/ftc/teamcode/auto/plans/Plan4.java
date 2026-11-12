package org.firstinspires.ftc.teamcode.auto.plans;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

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

import static org.firstinspires.ftc.teamcode.Config.*;
import org.firstinspires.ftc.teamcode.auto.Auto;
import org.firstinspires.ftc.teamcode.controllers.AutoAimController;
import org.firstinspires.ftc.teamcode.controllers.IntakeTransferController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

@Autonomous(name = "Plan 4", group = "Plans")
public class Plan4 extends LinearOpMode {

    private Follower follower;

    private IntakeTransferController intakeTransferController;
    private OuttakeController outtakeController;
    private AutoAimController autoAimController;
    private Alliance.Hive currentTargetHive = null;
    private boolean shouldAutoAim = false;
    private boolean gateOpen = false;
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();
    private boolean shooting = false;
    private int shootPart = 0;
    private int gateOpensDone = 0;

    private final Alliance alliance = currentAlliance == null ? Alliance.BLUE : currentAlliance;

    private final Alliance.Hive FRONT_CELL = alliance.TOP;
    private final Alliance.Hive REAR_CELL = alliance.BOTTOM;

    private final PoseFactory poseFactory = alliance == Alliance.BLUE
            ? PoseFactory.degrees()
            : PoseFactory.degrees().mirrorX(72).mirrorY(72);

    private final Pose start = poseFactory.of(85.3292, 5.1915, 90);
    private final Pose move1 = poseFactory.of(85.087, 11.8986, -92.4288);
    private final Pose move1Segment1Target = poseFactory.of(87, 57, 0);
    private final Pose move2 = poseFactory.of(132, 28, 83.2902);
    private final Pose move2Segment1Target = poseFactory.of(140, 96, 0);
    private final Pose move3 = poseFactory.of(115.4, 96.5859, -1.3644);
    private final Pose move3Segment1Target = poseFactory.of(140, 96, 0);
    private final Pose move4 = poseFactory.of(86.3095, 133.7184, 90.8468);
    private final Pose move4Segment1Target = poseFactory.of(87, 87, 0);
    private final Pose move5 = poseFactory.of(128, 134, -86.9178);
    private final Pose move5Segment1Target = poseFactory.of(135, 4, 0);
    private final Pose park = poseFactory.of(133.6, 26, -86.3588);
    private final Pose parkControl1 = poseFactory.of(138, 134, 0);
    private final Pose parkControl2 = poseFactory.of(138, 26, 0);
    private final Pose parkSegment1Target = poseFactory.of(135, 4, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> {
                    intakeTransferController.toggleForward();
                    outtakeController.close();
                }),
                parallel(
                        instant(() -> currentTargetHive = REAR_CELL),
                        follow(follower, move1())
                ),
                waitUntil(() -> autoTimer.seconds() >= Auto.partnerWaitS()),
                aimAndShoot(),
                follow(follower, move2()),
                waitMs(alliance == Alliance.RED ? 2000 : 1750),
                parallel(
                        instant(() -> intakeTransferController.flowerIntakeDown()),
                        follow(follower, move3())
                ),
                waitMs(Auto.FLOWER_INTAKE_MS),
                instant(() -> intakeTransferController.flowerIntakeUp()),
                parallel(
                        instant(() -> currentTargetHive = FRONT_CELL),
                        follow(follower, move4())
                ),
                aimAndShoot(),
                follow(follower, move5()),
                instant(() -> {
                    stopShooting();
                    closeGate();
                    intakeTransferController.flowerIntakeUp();
                }),
                follow(follower, park())
        );
    }

    private Command aimAndShoot() {
        return sequential(
                instant(() -> {
                    stepTimer.reset();
                    shootPart = 0;
                    gateOpensDone = 0;
                    shooting = true;
                }),
                waitUntil(() -> !shooting)
        );
    }

    private void openGate() {
        if (Auto.NECTAR_GATE_ONLY) {
            outtakeController.fireNectar();
        } else {
            outtakeController.fire();
        }
        gateOpen = true;
        gateTimer.reset();
    }

    private void closeGate() {
        outtakeController.close();
        gateOpen = false;
    }

    private void stopShooting() {
        shouldAutoAim = false;
        currentTargetHive = null;
    }

    private void updateShooting() {
        if (!shooting) return;
        if (shootPart == 0) {
            boolean settled = follower.distanceToEndpoint() < Auto.SETTLE_DISTANCE_IN;
            boolean tooLong = stepTimer.milliseconds() > Auto.SETTLE_TIMEOUT_MS;
            if (settled || tooLong) {
                shouldAutoAim = true;
                stepTimer.reset();
                shootPart = 1;
            }
        }
        if (shootPart == 1 && stepTimer.milliseconds() >= Auto.AIM_MS) {
            openGate();
            stepTimer.reset();
            shootPart = 2;
        }
        if (shootPart == 2 && stepTimer.milliseconds() >= Auto.SHOT_OPEN_MS) {
            closeGate();
            stepTimer.reset();
            shootPart = 3;
        }
        if (shootPart == 3 && stepTimer.milliseconds() >= Auto.SHOT_GAP_MS) {
            gateOpensDone = gateOpensDone + 1;
            stepTimer.reset();
            if (gateOpensDone < Math.max(1, Auto.GATE_OPENS_PER_SHOOT)) {
                openGate();
                shootPart = 2;
            } else {
                shootPart = 4;
            }
        }
        if (shootPart == 4 && stepTimer.milliseconds() >= Auto.SHOT_FINISH_MS) {
            stopShooting();
            shooting = false;
        }
    }

    private double shotVelocity() {
        if (!Auto.RAMP_ENABLED || !gateOpen) return Auto.SHOT_FLYWHEEL_VELOCITY;
        double msIntoRamp = gateTimer.milliseconds() - Auto.RAMP_START_MS;
        if (msIntoRamp <= 0) return Auto.SHOT_FLYWHEEL_VELOCITY;
        double fraction = 1;
        if (Auto.RAMP_DURATION_MS > 0) {
            fraction = Math.min(1, msIntoRamp / Auto.RAMP_DURATION_MS);
        }
        return Auto.SHOT_FLYWHEEL_VELOCITY + Auto.RAMP_AMOUNT * fraction;
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

        try {
            waitForStart();
            autoTimer.reset();
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();
                updateShooting();

                Pose robot = follower.pose();
                if (currentTargetHive != null) {
                    autoAimController.update(currentTargetHive, robot);
                }
                double targetVelocity = shotVelocity();
                outtakeController.flywheel.setVelocity(targetVelocity);
                outtakeController.flywheel.update();
                intakeTransferController.update();

                if (shouldAutoAim) {
                    ManualDrive.driveOrHold(follower, new DrivePowers(0, 0, autoAimController.rotation));
                }

                telemetry.addData("auto time", autoTimer.seconds());
                telemetry.addData("gate open", gateOpen);
                telemetry.addData("shoot step", shooting ? shootPart : -1);
                telemetry.addData("flywheel target", targetVelocity);
                telemetry.addData("flywheel velocity", outtakeController.flywheel.getVelocity());
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

    public Path move1() {
        return Paths.line(start, move1).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(move1Segment1Target).reverse()));
    }

    public Path move2() {
        return Paths.line(move1, move2).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(move2Segment1Target)));
    }

    public Path move3() {
        return Paths.line(move2, move3).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(move3Segment1Target)));
    }

    public Path move4() {
        return Paths.line(move3, move4).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(move4Segment1Target).reverse()));
    }

    public Path move5() {
        return Paths.line(move4, move5).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(move5Segment1Target)));
    }

    public Path park() {
        return Paths.curve(move5, parkControl1, parkControl2, park).heading(Interpolator.piecewise().until(1, Interpolator.facingPoint(parkSegment1Target)));
    }
}
