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

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import static org.firstinspires.ftc.teamcode.Config.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.LED;
import org.firstinspires.ftc.teamcode.prism.Color;
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

    RumbleController rumble;
    IntakeTransferController intakeTransfer;
    private Follower follower;
    RumbleGamepad driver;
    public boolean shouldAutoAim = false;

    public boolean fieldCentric = false; //false - robot centric | true - field centric

    private AutoAimController autoAim;
    private OuttakeController outtake;
    public Alliance alliance = currentAlliance;

    public Alliance.Hive activeHive = alliance.TOP;//defaults top red
    public LED prism;
    public Profile user;
    private double fieldCentricHeadingOffset = 0;
    public double speedMultiplier = 0.5;
    public boolean overrideDriver = false;

    public double odometryMultiplier = 1.152;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        driver = new RumbleGamepad(gamepad1);
        autoAim = new AutoAimController(hardwareMap);
        outtake = new OuttakeController(hardwareMap);
        rumble = new RumbleController(driver);
        intakeTransfer = new IntakeTransferController(hardwareMap);
        user = Drivers.activeDriver;
        prism = new LED(hardwareMap);
    }

    @Override
    public void start(){
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
        outtake.close();
        intakeTransfer.flowerIntakeUp();
        resetYaw();
    }

    @Override
    public void loop() {
        driver.update();

        //toggle team colors with start button
        if (driver.wasJustPressed(user.ALLIANCE_COLOR)) {
            alliance = alliance == Alliance.RED ? Alliance.BLUE : Alliance.RED;
        }

        if (alliance == Alliance.RED){
            driver.light(255,0,0); //show red light on gamepad
        }else{
            driver.light(0,0,255); //show blue light on gamepad
        }

        follower.update();
        Pose robot = follower.pose(); // gets robot pose
//        Pose robot = new Pose(follower.pose().x() * odometryMultiplier, follower.pose().y() * odometryMultiplier, follower.pose().heading());

        //reset yaw
        if (driver.wasJustPressed(user.RESET_YAW)) {
            resetYaw();
        }

        // relocalise button (SHOULD BE IMMEDIATELY AFTER POSE IS UPDATED)
        if (driver.wasJustPressed(user.REMOVE_DRIFT)) {
            follower.setPose(LOCALIZATION_RESET_POSITION); // overrides our pose with that ^
            robot = LOCALIZATION_RESET_POSITION;
            follower.hold(robot);//fixes hold issue
        }

        //AUTO AIM
        if (driver.wasJustPressed(user.TOGGLE_AUTOAIM)){
            shouldAutoAim = !shouldAutoAim;
        }

        activeHive = getTargetHive(alliance, robot);
        autoAim.update(activeHive, robot);//autoaim must update before outtake because it fetches limelight info!

        //OUTTAKE
        outtake.update(activeHive, robot);

        if (driver.isDown(user.FIRE)){
            outtake.fire();
        }
        if (driver.wasJustReleased(user.FIRE)){
            outtake.close();
        }

        //INTAKE & TRANSFER
        if (driver.wasJustPressed(user.INTAKE_FORWARD)){
            intakeTransfer.toggleForward();
        }
        if (driver.wasJustPressed(user.INTAKE_REVERSE)) {
            intakeTransfer.toggleReverse();
        }
        intakeTransfer.update();

        //FLOWER INTAKE
        if (driver.wasJustPressed(user.INTAKE_FLOWER_TOGGLE)){
            intakeTransfer.toggleFlowerIntake();
        }

        //DRIVE TRAIN
        DrivePowers powers;
        if (driver.wasJustPressed(user.FIELD_CENTRIC)){
            fieldCentric = !fieldCentric;
        }
        if(driver.wasJustPressed(user.TO_CLOSEST_RADIUS)){
            driveToHiveRadius(robot, activeHive.pose, 50);
            overrideDriver = true;
        }
        if (driver.wasJustPressed(user.TOGGLE_SPEED)){
            speedMultiplier = speedMultiplier == 1 ? 0.5 : 1;
        }
        double exponent = 3;
        double rotation = shouldAutoAim ? autoAim.rotation : RumbleGamepad.exponentialInput(-driver.rightX(), exponent) * speedMultiplier;
        double strafe = RumbleGamepad.exponentialInput(-driver.leftX(), exponent) * speedMultiplier;
        double throttle = RumbleGamepad.exponentialInput(driver.leftY(), exponent) * speedMultiplier;
        if (fieldCentric){
            powers = ManualDrive.fieldCentric(
                    throttle,
                    strafe,
                    rotation,
                    follower.pose().heading() - fieldCentricHeadingOffset
            );
        }else {
            powers = new DrivePowers(
                    throttle,
                    strafe,
                    rotation
            );
        }
        if(overrideDriver){
            if (!follower.following()) {
                overrideDriver = false;
            }
        }else{
            ManualDrive.driveOrHold(follower, powers);
        }

        telemetry.addData("Auto Aim", shouldAutoAim);
        telemetry.addData("Field Centric", fieldCentric);
        String allianceName = alliance == Alliance.BLUE ? "Blue" : "Red";
        telemetry.addData("Alliance", allianceName);
        String speedMode = speedMultiplier == 1 ? "Fast" : "Slow";
        telemetry.addData("Speed",speedMode);

        telemetry.addData("Distance", outtake.distance);
        telemetry.addData("Active Hive", activeHive.name);

        telemetry.addData("Flywheel Velocity", outtake.flywheel.getVelocity());
        telemetry.addData("Override Driver", overrideDriver);

        telemetry.addData("Robot X", robot.x());
        telemetry.addData("Robot Y", robot.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robot.heading()));

        //TELEMETRY
        if (DEBUG){
            telemetry.addData("Robot X", robot.x());
            telemetry.addData("Robot Y", robot.y());
            telemetry.addData("Target Heading", autoAim.calculateLocalizedHeading(activeHive, robot));
            telemetry.addData("Robot Heading", Math.toDegrees(robot.heading()));
            telemetry.addData("Intake Transfer Current", intakeTransfer.getCurrent());
            telemetry.addData("Flywheel Velocity", outtake.flywheel.getVelocity());
            telemetry.addData("Flywheels On", outtake.useFlywheel);
            telemetry.addData("Distance", outtake.distance);
            telemetry.addData("Intake Speed", intakeTransfer.getPower());
            telemetry.addLine("");
            telemetry.addLine("Limelight");
            telemetry.addData("TVisible", autoAim.limelight.ty);
            telemetry.addData("TX", autoAim.limelight.tx);
            telemetry.addLine("");
            telemetry.addLine("Active Hive");
            telemetry.addData("Active Hive", activeHive.name);
            telemetry.addData("AprilTag Visible", autoAim.isAprilTag());
            telemetry.addData("LED Alliance Color", allianceName);
        }

        //DRIVER FEEDBACK
        rumble.update(intakeTransfer.isOn());
        prism.update(autoAim.isAprilTag(),allianceName);
    }

    @Override
    public void stop(){
        outtake.stop();
        prism.turnOff();
    }

    public static Pose getClosestPointOnRadius(Pose robot, Pose hive, double radius) {
        double dx = robot.x() - hive.x();
        double dy = robot.y() - hive.y();

        double distance = Math.sqrt(dx * dx + dy * dy);

        //make sure distance is not 0
        if (distance < 0.001) {
            return new Pose(hive.x() + radius, hive.y(), robot.heading());
        }

        double x = hive.x() + (dx / distance) * radius;
        double y = hive.y() + (dy / distance) * radius;

        return new Pose(x, y, robot.heading());
    }

    private void driveToHiveRadius(Pose robot, Pose hive, double radius) {
        Pose target = getClosestPointOnRadius(robot, hive, radius);

        Path path = Paths.line(robot, target)
                .constant(robot.heading());

        follower.follow(path);
    }

    private Alliance.Hive getTargetHive(Alliance alliance, Pose robot) {return robot.y() > HIVE_BOUNDARY ? alliance.TOP : alliance.BOTTOM;}
    public void resetYaw() {fieldCentricHeadingOffset = follower.pose().heading();}
}