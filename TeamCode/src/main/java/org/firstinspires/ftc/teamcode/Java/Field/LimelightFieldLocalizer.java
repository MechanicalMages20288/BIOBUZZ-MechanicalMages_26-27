package org.firstinspires.ftc.teamcode.Java.Field;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Java.Teleop.LimelightPositionLM1;

/*
 * Field position from the Limelight's AprilTag view, used to CORRECT the Pinpoint pose.
 *
 * The Limelight is the main source of truth: whenever it sees tags and the reading passes the
 * checks below, the robot's pose is pulled to the Limelight's pose. The Pinpoint only carries
 * the pose between fixes (e.g. while the camera is facing away from the hive).
 *
 * A reading is IGNORED when:
 *   - no tags are in view, or the frame is old (camera lag),
 *   - the tags are too far away to be accurate,
 *   - the robot is turning or driving fast (blurry, laggy frames give wrong poses),
 *   - it jumps far from where the robot just was, right after a good fix (a bad frame).
 * After a long gap with no fixes, the first good reading is trusted fully (snap).
 * Otherwise readings are blended in, so one noisy frame doesn't make aiming twitch.
 *
 * Uses the same pipeline (uploaded at INIT) and frame conversion as LimelightPositionLM1.
 */
public class LimelightFieldLocalizer {

    // ---------- Filters (tune these) ----------
    public static double MAX_STALENESS_MS = 120;     // ignore frames older than this
    public static double MAX_TAG_DISTANCE_IN = 110;  // ignore tags farther than this (~9 ft)
    public static double MAX_TURN_RATE_DEG_S = 90;   // ignore readings while spinning faster
    public static double MAX_SPEED_IN_S = 45;        // ignore readings while driving faster
    public static double OUTLIER_JUMP_IN = 24;       // a reading this far from the pose is suspect...
    public static double OUTLIER_JUMP_DEG = 20;      // ...or this far off in heading...
    public static double TRUST_GAP_SECONDS = 1.5;    // ...unless there was no fix for this long
    public static double POSITION_BLEND = 0.35;      // how far to move toward each good reading (0-1)
    public static double HEADING_BLEND = 0.25;
    public static double MIN_WRITE_IN = 0.2;         // skip tiny corrections (saves Pinpoint writes)
    public static double MIN_WRITE_DEG = 0.3;

    private static final double METERS_TO_INCHES = 39.3701;

    private final Limelight3A limelight;
    private final boolean pipelineUploaded;
    private final ElapsedTime sinceFix = new ElapsedTime();
    private boolean hasFix = false;

    // Latest info, for telemetry.
    private Pose lastReading = null;
    private int lastTagCount = 0;
    private double lastTagDistanceIn = 0;
    private String status = "starting";
    private int acceptedCount = 0;
    private int rejectedCount = 0;

    public LimelightFieldLocalizer(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, LimelightPositionLM1.LIMELIGHT_NAME);
        boolean uploaded = false;
        if (LimelightPositionLM1.UPLOAD_PIPELINE_AT_INIT) {
            uploaded = limelight.uploadPipeline(LimelightPositionLM1.PIPELINE_JSON, LimelightPositionLM1.APRILTAG_PIPELINE);
        }
        pipelineUploaded = uploaded;
        limelight.pipelineSwitch(LimelightPositionLM1.APRILTAG_PIPELINE);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    /*
     * The Limelight's current field pose (Pedro frame, inches/radians), or null if it has no
     * usable reading. Doesn't apply the motion/outlier filters: use for INIT, when still.
     */
    public Pose readPose() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getBotposeTagCount() < 1) {
            lastTagCount = 0;
            return null;
        }
        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            return null;
        }
        lastTagCount = result.getBotposeTagCount();
        lastTagDistanceIn = result.getBotposeAvgDist() * METERS_TO_INCHES;
        if (result.getStaleness() > MAX_STALENESS_MS) {
            status = "old frame";
            return null;
        }
        Position m = botpose.getPosition().toUnit(DistanceUnit.METER);
        double rot = Math.toRadians(LimelightPositionLM1.LL_FRAME_ROTATION_DEG);
        double x = FieldLM1.CENTER + (Math.cos(rot) * m.x - Math.sin(rot) * m.y) * METERS_TO_INCHES;
        double y = FieldLM1.CENTER + (Math.sin(rot) * m.x + Math.cos(rot) * m.y) * METERS_TO_INCHES;
        double heading = wrap(botpose.getOrientation().getYaw(AngleUnit.RADIANS) + rot);
        lastReading = new Pose(x, y, heading);
        return lastReading;
    }

    /*
     * Pull the localizer's pose toward the Limelight's, if the reading is trustworthy.
     * turnRateDegS / speedInS are how fast the robot is moving right now (from odometry).
     * Returns true if a correction was applied.
     */
    public boolean correct(Localizer localizer, double turnRateDegS, double speedInS) {
        Pose reading = readPose();
        if (reading == null) {
            if (lastTagCount == 0) status = "no tags in view";
            return false;
        }
        if (lastTagDistanceIn > MAX_TAG_DISTANCE_IN) {
            return reject(String.format("tags too far (%.0f in)", lastTagDistanceIn));
        }
        if (Math.abs(turnRateDegS) > MAX_TURN_RATE_DEG_S) {
            return reject("turning too fast");
        }
        if (speedInS > MAX_SPEED_IN_S) {
            return reject("driving too fast");
        }

        Pose current = localizer.pose();
        double jumpIn = current.distance(reading);
        double jumpDeg = Math.abs(Math.toDegrees(wrap(reading.heading() - current.heading())));
        boolean recentFix = hasFix && sinceFix.seconds() < TRUST_GAP_SECONDS;
        if (recentFix && (jumpIn > OUTLIER_JUMP_IN || jumpDeg > OUTLIER_JUMP_DEG)) {
            return reject(String.format("jumped %.0f in / %.0f deg", jumpIn, jumpDeg));
        }

        // Snap after a gap (odometry may have drifted), otherwise blend in smoothly.
        double pBlend = recentFix ? POSITION_BLEND : 1.0;
        double hBlend = recentFix ? HEADING_BLEND : 1.0;
        Pose corrected = new Pose(
                current.x() + pBlend * (reading.x() - current.x()),
                current.y() + pBlend * (reading.y() - current.y()),
                wrap(current.heading() + hBlend * wrap(reading.heading() - current.heading())));

        boolean worthWriting = current.distance(corrected) > MIN_WRITE_IN
                || Math.abs(Math.toDegrees(wrap(corrected.heading() - current.heading()))) > MIN_WRITE_DEG;
        if (worthWriting) {
            localizer.setPose(corrected);
        }
        hasFix = true;
        sinceFix.reset();
        acceptedCount++;
        status = recentFix ? "tracking (blending)" : "fix (snapped)";
        return true;
    }

    private boolean reject(String reason) {
        rejectedCount++;
        status = "ignored: " + reason;
        return false;
    }

    // ---------- Info for telemetry ----------

    public boolean isConnected() {
        return limelight.isConnected();
    }

    public boolean pipelineUploaded() {
        return pipelineUploaded;
    }

    public String status() {
        return status;
    }

    public int tagCount() {
        return lastTagCount;
    }

    public double tagDistanceIn() {
        return lastTagDistanceIn;
    }

    public Pose lastReading() {
        return lastReading;
    }

    // Seconds since the last accepted Limelight correction (large if never).
    public double secondsSinceFix() {
        return hasFix ? sinceFix.seconds() : Double.POSITIVE_INFINITY;
    }

    public int acceptedCount() {
        return acceptedCount;
    }

    public int rejectedCount() {
        return rejectedCount;
    }

    public void stop() {
        limelight.stop();
    }

    private static double wrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
