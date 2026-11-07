package org.firstinspires.ftc.teamcode.Java.Field;

import com.pedropathing.math.Pose;

/*
 * Hands the robot's position from auto to teleop. The auto saves its pose here every loop;
 * the Red/Blue teleops read it at INIT. Static values survive between OpModes as long as the
 * Robot Controller app isn't restarted (a power cycle or app restart clears them).
 */
public final class PoseStorage {

    // Last pose saved by an auto, or null if no auto has run since the app started.
    public static Pose lastPose = null;

    // Which alliance that auto was run for.
    public static FieldLM1.Alliance alliance = null;

    public static void save(Pose pose, FieldLM1.Alliance a) {
        lastPose = pose;
        alliance = a;
    }

    // Use the saved pose once, then forget it so a later teleop doesn't reuse a stale one.
    public static void clear() {
        lastPose = null;
        alliance = null;
    }

    private PoseStorage() {
    }
}
