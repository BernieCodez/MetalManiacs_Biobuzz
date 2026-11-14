package org.firstinspires.ftc.teamcode.pedro;

import static org.firstinspires.ftc.teamcode.Config.*;

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

public class Constants {
    public static GoBildaPinpointDriver pinpoint;

    public static Follower create(HardwareMap h) {
        pinpoint = h.get(GoBildaPinpointDriver.class, PINPOINT);

        Follower follower =  new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );

        //add encoder compensation
        pinpoint.setEncoderResolution(17.269, DistanceUnit.MM);

        return follower;
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
        c.xPodOffset.set(2.6667091790146715);
        c.yPodOffset.set(-5.663497804656743);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3537240958115118);
                Controller secondaryTranslationalForward = Controller.proportional(0.1306916017204394);
                Controller primaryTranslationalLateral = Controller.proportional(0.6046041213462996);
                Controller secondaryTranslationalLateral = Controller.proportional(0.22338506751779852);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01190680803242371));
                c.brake.set(Controller.proportionalFeedforward(0.010120786827560152));

                c.headingFeedback.set(Controller.proportional(6.201515060913429));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.060036443929185286, 0.0037163361092962845));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08253121844031304, 0.05953101318410438));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0017312563285698697, 0.001509342956225645));

                c.maxAchievableForwardVelocity.set(80.94733803442028);
                c.maxAchievableStrafeVelocity.set(63.45823277121879);
                c.naturalForwardDeceleration.set(30.99255245872546);
                c.naturalStrafeDeceleration.set(76.81242623247438);
            }
    );
}