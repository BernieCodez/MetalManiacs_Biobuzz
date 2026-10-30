package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
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

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;

import org.firstinspires.ftc.teamcode.controllers.IntakeTransferController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import static org.firstinspires.ftc.teamcode.Config.*;

@Autonomous(name = "Auto2", group = "Autonomous")
public class Auto2 extends LinearOpMode {

    private Follower follower;

    private IntakeTransferController intakeTransferController;
    private OuttakeController outtakeController;

    private final Alliance alliance = currentAlliance;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private Pose pose(double x, double y, double heading) {
        if (alliance == Alliance.RED) {
            return poseFactory.of(x, y, heading);
        }
        return poseFactory.of(144 - x, 144 - y, heading + 180);
    }

    private final Pose start = pose(3.9781, 71.3295, 90);
    private final Pose goForward = pose(10.5575, 71.2624, -17.8332);
    private final Pose parkAtEnd = pose(9.7743, 121.3499, 90.8958);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                instant(() -> {
                    intakeTransferController.setPower(1);
                    intakeTransferController.flowerIntakeUp();
                    outtakeController.close();
                }),
                follow(follower, goForward()),
                follow(follower, parkAtEnd())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        intakeTransferController = new IntakeTransferController(hardwareMap);
        outtakeController = new OuttakeController(hardwareMap);
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

                telemetry.addData("auto time", autoTimer.seconds());
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

    public Path goForward() {
        return line(start, goForward).tangent();
    }

    public Path parkAtEnd() {
        return line(goForward, parkAtEnd).tangent();
    }
}
