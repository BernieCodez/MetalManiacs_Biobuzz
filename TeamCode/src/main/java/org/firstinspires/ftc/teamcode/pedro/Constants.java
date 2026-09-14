package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Algorithm;
import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-1.9579110934039738);
        c.yPodOffset.set(-6.565860538032112);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.32009636687101095);
                Controller secondaryTranslationalForward = Controller.proportional(0.11826705442639045);
                Controller primaryTranslationalLateral = Controller.proportional(0.40862398804703565);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15097564494932772);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01602527068429527));
                c.brake.set(Controller.proportionalFeedforward(0.01362148008165098));

                c.headingFeedback.set(Controller.proportional(4.133568606187627));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.041953028223185584, 0.005133194432925051));

                c.linearBrakeCoefficients.set(Matrix.diag(0.07175024116533511, 0.061660762119580347));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0013507782786527913, 0.0011016107973188234));

                c.maxAchievableForwardVelocity.set(65.94602798059717);
                c.maxAchievableStrafeVelocity.set(52.62496165440196);
                c.naturalForwardDeceleration.set(36.119688737440086);
                c.naturalStrafeDeceleration.set(71.47040874113426);
            }
    );

    public static Follower create(HardwareMap h) {
        Drivetrain drivetrain = new Mecanum(h, drivetrainConfig);
        Localizer localizer = new PinpointLocalizer(h, localizerConfig);
        Algorithm algorithm = new Foresight(foresightConfig);
        return new Follower(localizer, drivetrain, algorithm);
    }
}