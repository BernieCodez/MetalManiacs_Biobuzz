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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import static org.firstinspires.ftc.teamcode.Config.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

    public boolean fieldCentric = true; //false - robot centric | true - field centric

    private AutoAimController autoAim;
    private OuttakeController outtake;
    public Alliance alliance = currentAlliance;

    public Alliance.Hive activeHive = alliance.TOP;//defaults top red

    public Profile user;
    private double fieldCentricHeadingOffset = 0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        driver = new RumbleGamepad(gamepad1);
        autoAim = new AutoAimController(hardwareMap);
        outtake = new OuttakeController(hardwareMap);
        rumble = new RumbleController(driver);
        intakeTransfer = new IntakeTransferController(hardwareMap);
        user = Drivers.activeDriver;
    }

    @Override
    public void start(){
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
        outtake.close();
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

        //reset yaw
        if (driver.wasJustPressed(RumbleGamepad.Button.Y)) {
            resetYaw();
        }

        // relocalise button (SHOULD BE IMMEDIATELY AFTER POSE IS UPDATED)
        if (driver.wasJustPressed(user.REMOVE_DRIFT)) {
            robot = LOCALIZATION_RESET_POSITION;
            follower.setPose(robot); // overrides our pose with that ^
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
        if (driver.wasJustPressed(user.INTAKE_ON)){
            intakeTransfer.setPower(1);
        }
        if (driver.wasJustPressed(user.REVERSE_INTAKE)) {
            intakeTransfer.setPower(-1);
        }

        //FLOWER INTAKE
        if (driver.wasJustPressed(user.INTAKE_FLOWER_TOGGLE)){
            intakeTransfer.toggleFlowerIntake();
        }


        //DRIVE TRAIN
        DrivePowers powers;
        if (driver.wasJustPressed(user.FIELD_CENTRIC)){
            fieldCentric = !fieldCentric;
        }
        double exponent = 3;
        double rotation = shouldAutoAim ? autoAim.rotation : RumbleGamepad.exponentialInput(-driver.rightX(), exponent);
        double strafe = RumbleGamepad.exponentialInput(-driver.leftX(), exponent);
        double throttle = RumbleGamepad.exponentialInput(driver.leftY(), exponent);
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
        ManualDrive.driveOrHold(follower, powers);
//        follower.manual(powers);

        //TELEMETRY
        telemetry.addData("Robot X", robot.x());
        telemetry.addData("Robot Y", robot.y());
        telemetry.addData("Target Heading", autoAim.calculateLocalizedHeading(activeHive, robot));
        telemetry.addData("Robot Heading", Math.toDegrees(robot.heading()));
        telemetry.addData("Auto Aim", shouldAutoAim);
        telemetry.addData("Field Centric", fieldCentric);
        telemetry.addData("Flywheel Velocity", outtake.flywheel.getVelocity());
        telemetry.addData("Flywheels On", outtake.useFlywheel);
        telemetry.addData("Distance", outtake.distance);
        telemetry.addLine("");
        telemetry.addLine("Limelight");
        telemetry.addData("TVisible", autoAim.limelight.ty);
        telemetry.addData("TX", autoAim.limelight.tx);
        telemetry.addLine("");
        telemetry.addLine("Active Hive");
        telemetry.addData("Active Hive", activeHive.name);

        //DRIVER FEEDBACK
        rumble.update(intakeTransfer.isOn());
    }

    @Override
    public void stop(){
        outtake.stop();
    }

    private Alliance.Hive getTargetHive(Alliance alliance, Pose robot) {return robot.y() > HIVE_BOUNDARY ? alliance.TOP : alliance.BOTTOM;}
    public void resetYaw() {fieldCentricHeadingOffset = follower.pose().heading();}
}