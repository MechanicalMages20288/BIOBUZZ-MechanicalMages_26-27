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
        c.frontLeftName.set("leftFrontDrive");
        c.frontRightName.set("rightFrontDrive");
        c.backLeftName.set("leftBackDrive");
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
        c.xPodOffset.set(-0.0);
        c.yPodOffset.set(-0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    /*
     * Path-following tuning. NOT TUNED YET: replace this whole block with the Foresight Tuner's
     * generated code before running the Tests or any Pedro auto.
     */
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
    });

    // Builds the Pedro follower for this robot. Use this in autos: Constants.create(hardwareMap)
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
