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

import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.controllers.AutoAimController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

import java.util.Objects;

@TeleOp(name = "Competition Drive Code", group = "B - Competition")
public class CompetitionDriveCode extends OpMode {

    private Follower follower;
    RumbleGamepad driver;
    public String teamColor = "red";
    public boolean shouldAutoAim = true;

    public boolean fieldCentric = false; //false - robot centric | true - field centric

    private AutoAimController autoAim;
    private OuttakeController outtake;
    public Config.Hive activeHive = null;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        driver = new RumbleGamepad(gamepad1);
        autoAim = new AutoAimController(hardwareMap);
        outtake = new OuttakeController(hardwareMap);
    }

    @Override
    public void start(){
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
        outtake.close();
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
                    -driver.rightX(),
                    follower.pose().heading()
            );
        }else {
            powers = new DrivePowers(
                    driver.leftY(),
                    driver.leftX(),
                    -driver.rightX()
            );
        }
        ManualDrive.driveOrHold(follower, powers);

        // relocalise button
        if (driver.wasJustPressed(RumbleGamepad.Button.OPTION)) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            follower.setPose(cornerPose); // overrides our pose with that ^
        }

        follower.update();
        Pose robot = follower.pose(); // gets robot pose

        activeHive = getTargetHive(teamColor, robot);
        autoAim.update(activeHive, robot, shouldAutoAim, driver.rightX());//autoaim must update before outtake because it fetches limelight info!
        outtake.update(activeHive, robot);

        if (driver.isDown(RumbleGamepad.Button.RIGHT_BUMPER)){
            outtake.fire();
        }
        if (driver.wasJustReleased(RumbleGamepad.Button.RIGHT_BUMPER)){
            outtake.close();
        }

        //TELEMETRY
        telemetry.addData("Robot X", robot.x());
        telemetry.addData("Robot Y", robot.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robot.heading()));
    }

    @Override
    public void stop(){
        outtake.stop();
    }

    private Config.Hive getTargetHive(String teamColor, Pose robot) {
        boolean top = robot.y() > Config.HIVE_BOUNDARY;

        if (teamColor.equals("red")) {
            return top ? Config.Hive.RED_TOP : Config.Hive.RED_BOTTOM;
        } else {
            return top ? Config.Hive.BLUE_TOP : Config.Hive.BLUE_BOTTOM;
        }
    }
}