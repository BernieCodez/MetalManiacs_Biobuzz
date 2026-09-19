package org.firstinspires.ftc.teamcode.teleop; /**
 *  <DRIVER MANUAL>
 *
 *  --DRIVER CONTROLS--
 *
 *  [MOVEMENT]
 *  LEFT STICK Y = forward / backward
 *  RIGHT STICK X = turn
 *  DPAD UP       = drive speed up
 *  DPAD DOWN     = drive speed down
 *
 *  [ARM]
 *  RIGHT BUMPER (hold) = raise arm
 *  LEFT BUMPER (hold)  = lower arm
 *
 *  [HAND / GRIPPER]
 *  A = open gripper
 *  B = close gripper
 */

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.hardware.PushbotArm;
import org.firstinspires.ftc.teamcode.util.RumbleGamepad;

import java.util.List;

@TeleOp(name = "Pushbot Code", group = "A - TeleOP")
public class PushbotDriveCode extends OpMode {

    public PushbotArm armTuning;
    public RumbleGamepad driver;
    List<LynxModule> allHubs;

    // Hardware - expansion hub motor port 0/1/2, servo port 0/1
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor armMotor;
    Servo leftClaw;
    Servo rightClaw;
    CRServo tail;
    public boolean tailval;
    double driveSpeed = 1.0;

    ElapsedTime tailTimer = new ElapsedTime();
    boolean tailOn = false;

    public static final double HAND_OPEN = 0;
    public static final double HAND_CLOSED = 0.2;

    public static final double LEFT_HAND_OPEN = 0.2;
    public static final double LEFT_HAND_CLOSED = 0;

    @Override
    public void init() {

        driver = new RumbleGamepad(gamepad1);

        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
        tail = hardwareMap.get(CRServo.class,"tail");
        armTuning = new PushbotArm(hardwareMap);

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        leftClaw.setPosition(LEFT_HAND_CLOSED);
        rightClaw.setPosition(HAND_CLOSED);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void loop() {
        driver.update();

        driver.light(255,255,255,1);

        //adjust driver speed
        if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_UP)) {
            driveSpeed = Math.min(1.0, driveSpeed + 0.1);

        } else if (driver.wasJustPressed(RumbleGamepad.Button.DPAD_DOWN)) {
            driveSpeed = Math.max(0.1, driveSpeed - 0.1);
        }

        //drive
        double forward = driver.leftY();
        double turn = driver.rightX();

        double leftPower = Range.clip(forward - turn, -1.0, 1.0) * driveSpeed;
        double rightPower = Range.clip(forward + turn, -1.0, 1.0) * driveSpeed;

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        //arm
        int currentArmPos = armMotor.getCurrentPosition();
        if (driver.wasJustPressed(RumbleGamepad.Button.RIGHT_BUMPER)) {
            armTuning.targetPosition = Math.min(400,armTuning.targetPosition + 1); //clamp to max 400 pos
        } else if (driver.wasJustPressed(RumbleGamepad.Button.LEFT_BUMPER)) {
            armTuning.targetPosition = Math.max(0,armTuning.targetPosition -1); //clamp to min 0 pos
        }

        //claw
        if (driver.wasJustPressed(RumbleGamepad.Button.A)) {
            leftClaw.setPosition(LEFT_HAND_OPEN);
            rightClaw.setPosition(HAND_OPEN);
        } else if (driver.wasJustPressed(RumbleGamepad.Button.B)) {
            leftClaw.setPosition(LEFT_HAND_CLOSED);
            rightClaw.setPosition(HAND_CLOSED);
        }

        if (driver.wasJustPressed(RumbleGamepad.Button.Y)) {
            tailval = true;
            tailTimer.reset();
            tailOn = true;
            tail.setPower(1);
        }

        if (driver.wasJustReleased(RumbleGamepad.Button.Y)) {
            tailval = false;
            tail.setPower(0);
        }

        if (tailval && tailTimer.milliseconds() >= 300) {
            tailOn = !tailOn;
            tail.setPower(tailOn ? 1 : 0);
            tailTimer.reset();
        }

        armTuning.update();

        //our favorite telemetry :D
        telemetry.addData("Drive Speed", driveSpeed);
        telemetry.addData("Left Power", leftPower);
        telemetry.addData("Right Power", rightPower);
        telemetry.addData("Arm Position", currentArmPos);
        telemetry.addData("Current Position", armTuning.getArmPosition());
        telemetry.addData("Target Position",armTuning.targetPosition);
        telemetry.addData("Left Claw",leftClaw.getPosition());
        telemetry.addData("Right Claw",rightClaw.getPosition());
        telemetry.update();

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void stop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        armMotor.setPower(0);
    }
}
