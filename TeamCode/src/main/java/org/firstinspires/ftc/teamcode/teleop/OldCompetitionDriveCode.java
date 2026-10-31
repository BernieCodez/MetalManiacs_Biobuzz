package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "OLD Competition Drive Code", group = "B - Competition")
public class OldCompetitionDriveCode extends OpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    private boolean useFieldCentricDrive = true;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                );

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        if (gamepad1.a){
            useFieldCentricDrive = !useFieldCentricDrive;
        }

        if(gamepad1.y){
            resetYaw();
        }

        if (useFieldCentricDrive) {
            driveFieldRelative(forward, right, rotate);
        } else {
            driveRobotRelative(forward, right, rotate);
        }
    }

    private void driveFieldRelative(double forward, double right, double rotate) {

        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta - getHeading());

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        driveRobotRelative(newForward, newRight, rotate);
    }

    private void driveRobotRelative(double forward, double right, double rotate) {

        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double br = forward + right - rotate;
        double bl = forward - right + rotate;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(br), Math.abs(bl)))));

        double scale = 0.75;

        frontLeft.setPower(scale * fl / max);
        frontRight.setPower(scale * fr / max);
        backRight.setPower(scale * br / max);
        backLeft.setPower(scale * bl / max);
    }
    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetYaw() {
        imu.resetYaw();
    }
}
