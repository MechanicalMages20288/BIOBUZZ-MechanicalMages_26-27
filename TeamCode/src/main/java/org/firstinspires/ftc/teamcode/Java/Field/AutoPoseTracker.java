package org.firstinspires.ftc.teamcode.Java.Field;

import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Tracks the robot's position during an auto with the Pinpoint and saves it to PoseStorage,
 * so RedTeleopLM1 / BlueTeleopLM1 start from wherever the auto ended.
 *
 * Use it in any auto:
 *   init():      tracker = new AutoPoseTracker(hardwareMap);
 *   init_loop(): tracker.selectAlliance(gamepad1, telemetry);   // X = Blue, B = Red
 *   start():     tracker.start();
 *   loop():      tracker.update();
 *   stop():      tracker.update();
 *
 * The robot must start at FieldLM1.autoStartPose() for the chosen alliance.
 */
public class AutoPoseTracker {

    private final PinpointLocalizer localizer;
    private FieldLM1.Alliance alliance = FieldLM1.Alliance.RED;

    public AutoPoseTracker(HardwareMap hardwareMap) {
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
    }

    // Call in init_loop(): gamepad 1 X picks Blue, B picks Red. Shows the choice on telemetry.
    public void selectAlliance(Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.x) {
            alliance = FieldLM1.Alliance.BLUE;
        } else if (gamepad.b) {
            alliance = FieldLM1.Alliance.RED;
        }
        telemetry.addData("Alliance", "%s   (gamepad 1: X = Blue, B = Red)", alliance);
        Pose start = FieldLM1.autoStartPose(alliance);
        telemetry.addData("Start spot", "x %.0f, y %.0f, heading %.0f deg",
                start.x(), start.y(), Math.toDegrees(start.heading()));
    }

    // Call in start(): sets the starting position for the chosen alliance.
    public void start() {
        localizer.setPose(FieldLM1.autoStartPose(alliance));
        PoseStorage.save(localizer.pose(), alliance);
    }

    // Call every loop (and in stop()): updates the position and saves it for teleop.
    public void update() {
        localizer.update();
        PoseStorage.save(localizer.pose(), alliance);
    }

    public Pose pose() {
        return localizer.pose();
    }

    public FieldLM1.Alliance alliance() {
        return alliance;
    }
}
