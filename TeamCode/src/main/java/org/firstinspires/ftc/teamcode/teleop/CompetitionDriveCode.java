/**
 *  <DRIVER MANUAL>
 *
 *  --DRIVER CONTROLS--
 *
 *  [MOVEMENT]
 *  LEFT STICK Y = forward / backward
 *  RIGHT STICK X = turn
 *  WE CAN POTENTIALLY USE DPADS FOR SNAP ROTATION TO 90 DEGREE ANGLES
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

import java.util.Objects;

@TeleOp(name = "Competition Drive Code", group = "B - Competition")
public class CompetitionDriveCode extends OpMode {

    private Follower follower;
    RumbleGamepad driver;
    public String teamColor = "red";

    public boolean fieldCentric = false; //false - robot centric | true - field centric

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        driver = new RumbleGamepad(gamepad1);
    }

    @Override
    public void start(){
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
    }

    @Override
    public void loop() {
        driver.update();

        if (driver.wasJustPressed(RumbleGamepad.Button.START)){ //toggle team colors with start button
            teamColor = teamColor.equals("red") ? "blue" : "red";
        }

        if (Objects.equals(teamColor, "red")){
            driver.light(255,0,0); //show red light on gamepad
        }else{
            driver.light(0,0,255); //show blue light on gamepad
        }

        //DRIVE TRAIN
        DrivePowers powers;
        if (driver.wasJustPressed((RumbleGamepad.Button.LEFT_STICK))){
            fieldCentric = !fieldCentric;
        }
        if (fieldCentric){
            powers = ManualDrive.fieldCentric(
                    driver.leftY(),
                    driver.leftX(),
                    driver.rightX(),
                    follower.pose().heading()
            );
        }else {
            powers = new DrivePowers(
                    driver.leftY(),
                    driver.leftX(),
                    driver.rightX()
            );
        }
        ManualDrive.driveOrHold(follower, powers);

        // relocalise button
        if (driver.wasJustPressed(RumbleGamepad.Button.START)) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            follower.setPose(cornerPose); // overrides our pose with that ^
        }

        follower.update();

        //TELEMETRY
        Pose robot = follower.pose(); // gets robot pose
        telemetry.addData("Robot X", robot.x());
        telemetry.addData("Robot Y", robot.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robot.heading()));
    }
}