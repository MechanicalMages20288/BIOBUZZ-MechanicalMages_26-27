package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/*
 * Tuning procedures shown on the AutoTune page (http://192.168.43.1:10158).
 * Run them in this order: Mecanum, Pinpoint, Foresight, then Tests.
 */
public class Tuning {

    // 1. Checks each drive motor's direction.
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    // 2. Measures the Pinpoint pod type, directions and offsets.
    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    // 3. Measures speed, braking and correction values for path following.
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                h -> new PinpointLocalizer(h, Constants.localizerConfig),
                h -> new Mecanum(h, Constants.drivetrainConfig)
        );
    }

    // 4. Localization, hold, line and curve tests to check the tuning.
    @Tuner
    public static Procedure tests() {
        return new Tests(
                h -> new Mecanum(h, Constants.drivetrainConfig),
                h -> new PinpointLocalizer(h, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig)
        );
    }
}
