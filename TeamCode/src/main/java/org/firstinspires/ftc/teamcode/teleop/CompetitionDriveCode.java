package org.firstinspires.ftc.teamcode.teleop;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;


import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

//Make sure this matches your file location

import com.pedropathing.math.Pose;
@TeleOp(name = "Example TeleOp")


public class CompetitionDriveCode extends OpMode {
    RumbleGamepad driver;
        private Follower follower;
        @Override
        public void init() {
            driver = new RumbleGamepad(gamepad1)
            follower = Constants.create(hardwareMap);
        }
        @Override
        public void loop() {
            DrivePowers powers = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    follower.pose().heading()
            );
            follower.manual(powers);

            // relocalise button
            if (gamepad1.startWasPressed) {
                Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
                // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
                follower.setPose(cornerPose); // overrides our pose
            }

            follower.update();
            Pose robotPose = follower.pose(); // returns a Pose object
            telemetry.addData("Robot X", robotPose.x());
            telemetry.addData("Robot Y", robotPose.y());
            telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
            // Math.toDegrees() is a built-in java method
        }
    // in your autonomous


    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }
    @Override
    public void start(){
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
    }
}
