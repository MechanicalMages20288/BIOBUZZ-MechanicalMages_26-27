package org.firstinspires.ftc.teamcode.Java.Teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Java.Field.LimelightFieldLocalizer;

/*
 * Shared code for LimelightOnlyTeleopRedLM1 / LimelightOnlyTeleopBlueLM1.
 *
 * Same controls and features as the other LM1 teleops (tap-A auto-aim, X cell override, auto
 * flywheel speed, fire feedforward), but the position comes ONLY from the Limelight. The
 * Pinpoint is never opened or read.
 *
 * WHEN THE TAGS ARE LOST
 *   0 to HOLD_SECONDS (1.5 s)  the last Limelight position is held. Auto speed keeps using it.
 *                              Auto-aim stops (it can't steer by a frozen heading).
 *   after HOLD_SECONDS         NO POSITION: auto-aim does nothing and the flywheel uses the
 *                              manual D-pad speed until the tags are seen again.
 *
 * NOT AVAILABLE (they need odometry to carry the position):
 *   - D-pad left/right corner relocalization: does nothing here.
 *   - Position handoff from the auto: the position starts from the first tag sighting instead.
 *
 * Readings are only rejected if the tags are too far away, or if one frame jumps far from the
 * last good reading (a bad frame). There's no odometry, so there's no "moving too fast" check.
 */
public abstract class LimelightOnlyAimTeleopLM1 extends AimTeleopLM1 {

    // Keep the last Limelight position this long after the tags are lost.
    public static double HOLD_SECONDS = 1.5;

    // Auto-aim only steers by a reading at most this old (seconds).
    public static double AIM_MAX_AGE_SECONDS = 0.25;

    // Reject a single reading that jumps this far from the last good one (within JUMP_WINDOW).
    public static double JUMP_REJECT_IN = 18;
    public static double JUMP_REJECT_DEG = 30;
    public static double JUMP_WINDOW_SECONDS = 0.3;

    private LimelightFieldLocalizer limelight;
    private Pose lastGood = null;
    private final ElapsedTime sinceGood = new ElapsedTime();
    private String status = "no tags seen yet";
    private int usedCount = 0;
    private int rejectedCount = 0;

    @Override
    protected boolean usePinpoint() {
        return false;
    }

    @Override
    public void init() {
        super.init();
        limelight = new LimelightFieldLocalizer(hardwareMap);
    }

    @Override
    public void init_loop() {
        updatePose();
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Limelight", "%s, pipeline %s",
                limelight.isConnected() ? "connected" : "NOT CONNECTED",
                limelight.pipelineUploaded() ? "uploaded" : "not uploaded (using saved)");
        Pose p = currentPose();
        telemetry.addData("Position", p == null ? "NO POSITION (point the back at the hive)"
                : String.format("x %.1f, y %.1f, heading %.0f deg", p.x(), p.y(), Math.toDegrees(p.heading())));
        telemetry.update();
    }

    @Override
    protected void updatePose() {
        Pose reading = limelight.readPose();
        if (reading == null) {
            status = limelight.tagCount() == 0 ? "no tags in view" : "old frame";
            return;
        }
        if (limelight.tagDistanceIn() > LimelightFieldLocalizer.MAX_TAG_DISTANCE_IN) {
            reject(String.format("tags too far (%.0f in)", limelight.tagDistanceIn()));
            return;
        }
        if (lastGood != null && sinceGood.seconds() < JUMP_WINDOW_SECONDS) {
            double jumpIn = lastGood.distance(reading);
            double jumpDeg = Math.abs(Math.toDegrees(wrap(reading.heading() - lastGood.heading())));
            if (jumpIn > JUMP_REJECT_IN || jumpDeg > JUMP_REJECT_DEG) {
                reject(String.format("jumped %.0f in / %.0f deg", jumpIn, jumpDeg));
                return;
            }
        }
        lastGood = reading;
        sinceGood.reset();
        usedCount++;
        status = "tracking";
    }

    private void reject(String reason) {
        rejectedCount++;
        status = "ignored: " + reason;
    }

    // Last good Limelight position, or null if none within HOLD_SECONDS.
    @Override
    protected Pose currentPose() {
        if (lastGood == null || sinceGood.seconds() > HOLD_SECONDS) {
            return null;
        }
        return lastGood;
    }

    // No odometry to carry a manually set position, so the corner buttons / auto handoff do nothing.
    @Override
    protected void setRobotPose(Pose pose) {
    }

    @Override
    protected boolean poseGoodForAiming() {
        return lastGood != null && sinceGood.seconds() <= AIM_MAX_AGE_SECONDS;
    }

    @Override
    protected void extraTelemetry() {
        telemetry.addLine();
        String source;
        if (lastGood == null) {
            source = "NO POSITION - no tags seen yet, point the back at the hive";
        } else if (sinceGood.seconds() <= AIM_MAX_AGE_SECONDS) {
            source = "LIMELIGHT";
        } else if (sinceGood.seconds() <= HOLD_SECONDS) {
            source = String.format("LIMELIGHT (holding last position, tags lost %.1f s ago)", sinceGood.seconds());
        } else {
            source = String.format("NO POSITION - tags lost %.0f s ago (manual speed, no aim)", sinceGood.seconds());
        }
        telemetry.addData("Position source", source);
        telemetry.addData("Limelight", "%s  (%d tag%s, %.0f in)", status,
                limelight.tagCount(), limelight.tagCount() == 1 ? "" : "s", limelight.tagDistanceIn());
        telemetry.addData("Limelight readings", "%d used, %d ignored", usedCount, rejectedCount);
    }

    @Override
    public void stop() {
        super.stop();
        limelight.stop();
    }

    private static double wrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
