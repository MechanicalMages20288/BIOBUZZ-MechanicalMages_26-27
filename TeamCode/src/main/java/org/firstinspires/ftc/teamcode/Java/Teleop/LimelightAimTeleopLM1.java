package org.firstinspires.ftc.teamcode.Java.Teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Java.Field.LimelightFieldLocalizer;

/*
 * Shared code for LimelightTeleopRedLM1 / LimelightTeleopBlueLM1.
 *
 * Same controls and features as RedTeleopLM1 / BlueTeleopLM1 (see AimTeleopLM1 and
 * BasicTeleopLM1): tap-A auto-aim, X cell override, auto flywheel speed, corner resets.
 *
 * The difference: the robot's position comes from the LIMELIGHT. Every loop where the camera
 * sees AprilTags and the reading passes the safety checks (LimelightFieldLocalizer), the pose
 * is corrected to match it. The Pinpoint odometry only carries the pose in between fixes, e.g.
 * while the robot faces away from the hive, so you can swing roughly toward the hive; once the
 * camera sees the tags again, the Limelight takes over.
 *
 * Telemetry "Position source" shows which one is in charge right now.
 */
public abstract class LimelightAimTeleopLM1 extends AimTeleopLM1 {

    // After this long without a Limelight fix, telemetry warns that it's odometry only.
    public static double ODOMETRY_ONLY_WARNING_SECONDS = 3;

    private LimelightFieldLocalizer limelight;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private Pose previousPose = null;

    @Override
    public void init() {
        super.init();
        limelight = new LimelightFieldLocalizer(hardwareMap);
    }

    @Override
    public void init_loop() {
        // If the camera can see tags during INIT, start from the Limelight's position.
        Pose seen = limelight.readPose();
        if (seen != null) {
            startPose = seen;
            poseSource = String.format("LIMELIGHT (%d tag%s)", limelight.tagCount(), limelight.tagCount() == 1 ? "" : "s");
        }
        telemetry.addData("Limelight", "%s, pipeline %s",
                limelight.isConnected() ? "connected" : "NOT CONNECTED",
                limelight.pipelineUploaded() ? "uploaded" : "not uploaded (using saved)");
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        previousPose = localizer.pose();
        loopTimer.reset();
    }

    @Override
    protected void correctPose() {
        // How fast the robot is moving right now (from odometry), so the Limelight can skip
        // blurry/laggy frames taken while turning or driving fast.
        Pose now = localizer.pose();
        double dt = Math.max(loopTimer.seconds(), 1e-3);
        loopTimer.reset();
        double turnRateDegS = 0;
        double speedInS = 0;
        if (previousPose != null) {
            turnRateDegS = Math.toDegrees(wrap(now.heading() - previousPose.heading())) / dt;
            speedInS = now.distance(previousPose) / dt;
        }

        limelight.correct(localizer, turnRateDegS, speedInS);

        // Measure motion from the corrected pose, so a correction isn't counted as movement.
        previousPose = localizer.pose();
    }

    @Override
    protected void extraTelemetry() {
        telemetry.addLine();
        double since = limelight.secondsSinceFix();
        String source;
        if (since < 0.3) {
            source = "LIMELIGHT";
        } else if (since < ODOMETRY_ONLY_WARNING_SECONDS) {
            source = String.format("odometry (last Limelight fix %.1f s ago)", since);
        } else {
            source = Double.isInfinite(since)
                    ? "ODOMETRY ONLY - no Limelight fix yet, point the back at the hive"
                    : String.format("ODOMETRY ONLY - no Limelight fix for %.0f s", since);
        }
        telemetry.addData("Position source", source);
        telemetry.addData("Limelight", "%s  (%d tag%s, %.0f in)", limelight.status(),
                limelight.tagCount(), limelight.tagCount() == 1 ? "" : "s", limelight.tagDistanceIn());
        telemetry.addData("Limelight fixes", "%d used, %d ignored",
                limelight.acceptedCount(), limelight.rejectedCount());
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
