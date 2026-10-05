/**
 *  <DRIVER MANUAL>
 *  *
 *  *  --DRIVER CONTROLS--
 *  *
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

import static org.firstinspires.ftc.teamcode.Config.*;

import org.firstinspires.ftc.teamcode.util.Drivers;
import org.firstinspires.ftc.teamcode.Profile;
import org.firstinspires.ftc.teamcode.controllers.AutoAimController;
import org.firstinspires.ftc.teamcode.controllers.IntakeTransferController;
import org.firstinspires.ftc.teamcode.controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.controllers.RumbleController;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.OpModeStorage;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

@TeleOp(name = "Competition Drive Code", group = "B - Competition")
public class CompetitionDriveCode extends OpMode {

    IntakeTransferController intake;

    RumbleController rumble;
    IntakeTransferController intakeTransfer;
    private Follower follower;
    RumbleGamepad driver;
    public boolean shouldAutoAim = true;

    public boolean fieldCentric = false; //false - robot centric | true - field centric

    private AutoAimController autoAim;
    private OuttakeController outtake;
    public Alliance alliance = Alliance.RED;

    public Alliance.Hive activeHive = alliance.TOP;//defaults top red

    public Profile user;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        driver = new RumbleGamepad(gamepad1);
        autoAim = new AutoAimController(hardwareMap);
        outtake = new OuttakeController(hardwareMap);
        user = Drivers.Ahmed;
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

        if (driver.wasJustPressed(user.ALLIANCE_COLOR)){ //toggle team colors with start button
            alliance = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
        }

        if (alliance == Alliance.RED){
            driver.light(255,0,0); //show red light on gamepad
        }else{
            driver.light(0,0,255); //show blue light on gamepad
        }

        //DRIVE TRAIN
        DrivePowers powers;
        if (driver.wasJustPressed((user.FIELD_CENTRIC))){
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
        if (driver.wasJustPressed(user.REMOVE_DRIFT)) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            follower.setPose(cornerPose); // overrides our pose with that ^
        }

        follower.update();
        Pose robot = follower.pose(); // gets robot pose

        activeHive = getTargetHive(alliance, robot);
        autoAim.update(activeHive, robot, shouldAutoAim, driver.rightX());//autoaim must update before outtake because it fetches limelight info!
        outtake.update(activeHive, robot);

        if (driver.isDown(RumbleGamepad.Trigger.RIGHT_TRIGGER)){
            outtake.fire();
        }
        if (driver.wasJustReleased(RumbleGamepad.Trigger.RIGHT_TRIGGER)){
            outtake.close();
        }

        //TELEMETRY
        telemetry.addData("Robot X", robot.x());
        telemetry.addData("Robot Y", robot.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robot.heading()));

            rumble.update(intakeTransfer.on());

    }

    @Override
    public void stop(){
        outtake.stop();
    }

    private Alliance.Hive getTargetHive(Alliance alliance, Pose robot) {return robot.y() > HIVE_BOUNDARY ? alliance.TOP : alliance.BOTTOM;}


}