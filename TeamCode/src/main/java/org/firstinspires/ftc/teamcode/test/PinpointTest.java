//
//package org.firstinspires.ftc.teamcode.test;
//
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//@TeleOp(name = "Pinpoint Test", group = "C - Util")
//public class PinpointTest extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//
//        // Change this name if your hardware configuration uses another name.
//        GoBildaPinpointDriver pinpoint =
//                hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//
//        // Configure these to match your existing Pinpoint setup.
//        pinpoint.setEncoderResolution(
//                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
//        );
//
//        // Set the same directions and offsets as your robot configuration.
//        // Example only: these directions may not match your robot.
//        pinpoint.setEncoderDirections(
//                GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.FORWARD
//        );
//
//        // Initialize the IMU and reset the reported position.
//        pinpoint.resetPosAndIMU();
//
//        telemetry.addLine("PINPOINT TEST");
//        telemetry.addLine("Robot must be lifted off the ground.");
//        telemetry.addLine("Spin the odometry wheels by hand.");
//        telemetry.addLine("No drive motors will be powered.");
//        telemetry.update();
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            // Refresh Pinpoint readings.
//            pinpoint.update();
//
//            telemetry.addLine("=== POSITION ===");
//            telemetry.addData("X (in)", "%.3f", pinpoint.getPosX());
//            telemetry.addData("Y (in)", "%.3f", pinpoint.getPosY());
//            telemetry.addData("Heading (deg)",
//                    "%.3f", Math.toDegrees(pinpoint.getHeading()));
//
//            telemetry.addLine("\n=== VELOCITY ===");
//            telemetry.addData("X velocity (in/s)",
//                    "%.3f", pinpoint.getVelX());
//            telemetry.addData("Y velocity (in/s)",
//                    "%.3f", pinpoint.getVelY());
//            telemetry.addData("Heading velocity (deg/s)",
//                    "%.3f", Math.toDegrees(pinpoint.getHeadingVelocity()));
//
//            telemetry.addLine("\n=== RAW ENCODERS ===");
//            telemetry.addData("X encoder (ticks)",
//                    pinpoint.getEncoderX());
//            telemetry.addData("Y encoder (ticks)",
//                    pinpoint.getEncoderY());
//
//            telemetry.update();
//
//            idle();
//        }
//    }
//}
