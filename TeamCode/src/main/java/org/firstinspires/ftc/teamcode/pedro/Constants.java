package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(3.286590576171875);
        c.yPodOffset.set(-6.828651127852793);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3648238695180691);
                Controller secondaryTranslationalForward = Controller.proportional(0.1347926715135965);
                Controller primaryTranslationalLateral = Controller.proportional(0.5577918366651946);
                Controller secondaryTranslationalLateral = Controller.proportional(0.2060891791754141);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01449355909579631));
                c.brake.set(Controller.proportionalFeedforward(0.012319525231426863));

                c.headingFeedback.set(Controller.proportional(6.634232950414453));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04507977699954571, 0.006427169811723469));

                c.linearBrakeCoefficients.set(Matrix.diag(0.08630438744485851, 0.050358876549572884));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0011773630008750863, 0.0015913102986951748));

                c.maxAchievableForwardVelocity.set(72.74681471069005);
                c.maxAchievableStrafeVelocity.set(57.09632333288541);
                c.naturalForwardDeceleration.set(43.034491673997024);
                c.naturalStrafeDeceleration.set(71.1687040466868);
            }
    );
}