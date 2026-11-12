package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * Pedro Pathing 3 robot constants.
 *
 * Each block below gets REPLACED by the code AutoTune generates (its "Java" tab) as you tune:
 *   1. drivetrainConfig  <- Mecanum Tuner
 *   2. localizerConfig   <- Pinpoint Tuner
 *   3. foresightConfig   <- Foresight Tuner
 * Tune in that order. AutoTune runs on the robot at http://192.168.43.1:10158
 */
public class Constants {

    /*
     * Drivetrain. Motor names match the teleop. The directions are the teleop's current ones;
     * the Mecanum Tuner checks each motor and generates the correct values.
     */
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        // Left front/back are swapped on purpose: on this robot "leftFrontDrive" is physically
        // the back-left wheel and vice versa (same fix as the teleop's strafe signs).
        c.frontLeftName.set("leftBackDrive");
        c.frontRightName.set("rightFrontDrive");
        c.backLeftName.set("leftFrontDrive");
        c.backRightName.set("rightBackDrive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });


    /*
     * goBILDA Pinpoint with 2 odometry pods. "pinpoint" must match its name in the robot config.
     * Pod type, directions and offsets are PLACEHOLDERS: the Pinpoint Tuner measures them.
     */
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        // Measured by hand (the tuner gave 6.34 / -5.32). Signs kept from the tuner:
        // forward pod 4 in LEFT of center (+), sideways pod 4.75 in BEHIND center (-).
        c.xPodOffset.set(4.0);
        c.yPodOffset.set(-4.75);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /*
     * Path-following tuning. NOT TUNED YET: replace this whole block with the Foresight Tuner's
     * generated code before running the Tests or any Pedro auto.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.27614529289169076);
                Controller secondaryTranslationalForward = Controller.proportional(0.10202830698535748);
                Controller primaryTranslationalLateral = Controller.proportional(0.3246399164180581);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11994577457814452);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014013777970952181));
                c.brake.set(Controller.proportionalFeedforward(0.011911711275309354));

                c.headingFeedback.set(Controller.proportional(4.66745592342899));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05442268971467273, 0.0039038575156050714));

                c.linearBrakeCoefficients.set(Matrix.diag(0.060777013122223604, 0.008167657484736975));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002197542281161376, 2.1619003280796666E-4));

                c.maxAchievableForwardVelocity.set(36.51637034230535);
                c.maxAchievableStrafeVelocity.set(33.33967787444321);
                c.naturalForwardDeceleration.set(42.729062260569826);
                c.naturalStrafeDeceleration.set(65.16229392389816);
            }
    );

    // Builds the Pedro follower for this robot. Use this in autos: Constants.create(hardwareMap)
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
