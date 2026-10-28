package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
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
import static org.firstinspires.ftc.teamcode.Config.*;

public class Constants {
    public static GoBildaPinpointDriver pinpoint;

    public static Follower create(HardwareMap h) {
        pinpoint = h.get(GoBildaPinpointDriver.class, PINPOINT);

        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(FRONT_LEFT_WHEEL);
        c.frontRightName.set(FRONT_RIGHT_WHEEL);
        c.backLeftName.set(BACK_LEFT_WHEEL);
        c.backRightName.set(BACK_RIGHT_WHEEL);
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(PINPOINT);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(4.733866894339013);
        c.yPodOffset.set(-6.516680980291893);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.34122003270745976);
                Controller secondaryTranslationalForward = Controller.proportional(0.1260716845182118);
                Controller primaryTranslationalLateral = Controller.proportional(0.6287743921040159);
                Controller secondaryTranslationalLateral = Controller.proportional(0.23231533672124538);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012917116710001987));
                c.brake.set(Controller.proportionalFeedforward(0.01097954920350169));

                c.headingFeedback.set(Controller.proportional(5.689184401015181));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0498054241224866, 0.004751705595692428));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11505582907670303, 0.0626932615008151));
                c.quadraticBrakeCoefficients.set(Matrix.diag(7.923267579445406E-4, 0.0013750598958739964));

                c.maxAchievableForwardVelocity.set(78.15419345797554);
                c.maxAchievableStrafeVelocity.set(58.74412751895481);
                c.naturalForwardDeceleration.set(38.398025035981114);
                c.naturalStrafeDeceleration.set(75.70853755564129);
            }
    );
}