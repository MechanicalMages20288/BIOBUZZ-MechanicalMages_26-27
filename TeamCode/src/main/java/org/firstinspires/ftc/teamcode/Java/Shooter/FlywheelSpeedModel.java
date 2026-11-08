package org.firstinspires.ftc.teamcode.Java.Shooter;

/*
 * Picks the flywheel speed (ticks/s) for a shot from the robot's distance to the hive cell,
 * using projectile physics.
 *
 * HOW IT WORKS
 *   The ball leaves the launcher at a fixed angle (LAUNCH_ANGLE_DEG) and height (LAUNCH_HEIGHT_IN)
 *   and has to come down into the cell at TARGET_HEIGHT_IN. For each distance, physics gives the
 *   exit speed needed. We don't know exactly how much of the flywheel's surface speed the ball
 *   gets, so the model is scaled to ONE known-good shot: CALIBRATION_DISTANCE_IN at
 *   CALIBRATION_VELOCITY. Every other distance is that speed times (needed exit speed there /
 *   needed exit speed at the calibration spot).
 *
 * DISTANCE: the "Distance to cell" shown on the Red/Blue teleop telemetry (robot center to cell
 * center, in inches). RANGE_OFFSET_IN is subtracted to get the launcher-to-cell range, since the
 * launcher is at the back of the robot, not its center.
 *
 * TO TUNE (most important first):
 *   1. LAUNCH_ANGLE_DEG: measure the ball's exit angle up from the floor.
 *   2. TARGET_HEIGHT_IN: measure the height of the tipped-up cell's opening.
 *   3. CALIBRATION_DISTANCE_IN: park at your best spot (1600 scores ~90%), read
 *      "Distance to cell" on telemetry, and put that number here.
 */
public final class FlywheelSpeedModel {

    /*
     * Ball exit angle, up from the floor. The hood measures ~82 deg, but shots behaved like a
     * flatter launch (close shots short, far shots long at 82), so the model uses 78 deg.
     */
    public static double LAUNCH_ANGLE_DEG = 78;

    // Height the ball leaves the flywheel.
    public static double LAUNCH_HEIGHT_IN = 7;

    // Height the ball needs to reach at the tipped-up cell (measured by the team: about 58 in).
    public static double TARGET_HEIGHT_IN = 58;

    // Robot center to launcher, along the robot (launcher is at the back).
    public static double RANGE_OFFSET_IN = 9;

    // Known-good shot: 30 in from the goal (+9 in to robot center) at about 1550 ticks/s.
    public static double CALIBRATION_DISTANCE_IN = 39;
    public static double CALIBRATION_VELOCITY = 1550;

    /*
     * Limits on the auto speed. A goBILDA 6000 RPM bare motor tops out around 2800 ticks/s
     * with no load, so stay below that.
     */
    // Blanket scale on top of the model, applied to every distance. 1.0 = off.
    // Was 1.05 (+5%); the team then asked for 93% of that: 1.05 x 0.93 = 0.9765.
    public static double SPEED_MULTIPLIER = 0.9765;

    /*
     * Near/far correction from testing: shots closer than the split were slightly short,
     * farther ones overshot. Distance here is LAUNCHER to goal (telemetry "Distance to cell"
     * minus RANGE_OFFSET_IN). The change is spread over SPLIT_BLEND_IN so the flywheel doesn't
     * jump when you drive across the line.
     */
    public static double NEAR_FAR_SPLIT_IN = 18;
    public static double SPLIT_BLEND_IN = 4;     // blend from 16 to 20 in
    public static double NEAR_MULTIPLIER = 1.07; // +7% when close
    public static double FAR_MULTIPLIER = 0.93;  // -7% when far

    public static double MIN_VELOCITY = 1200;
    public static double MAX_VELOCITY = 2600;

    private static final double GRAVITY_IN_PER_S2 = 386.09;

    /*
     * Flywheel target (ticks/s) for a shot from this distance (robot center to cell, inches).
     * Too close to reach the cell at this angle -> MAX_VELOCITY.
     */
    public static int velocityForDistance(double distanceIn) {
        double needed = exitSpeed(distanceIn - RANGE_OFFSET_IN);
        double atCalibration = exitSpeed(CALIBRATION_DISTANCE_IN - RANGE_OFFSET_IN);

        double velocity;
        if (Double.isNaN(needed) || Double.isNaN(atCalibration)) {
            velocity = MAX_VELOCITY;
        } else {
            velocity = CALIBRATION_VELOCITY * needed / atCalibration * SPEED_MULTIPLIER
                    * nearFarMultiplier(distanceIn);
        }
        velocity = Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, velocity));
        return (int) Math.round(velocity);
    }

    /*
     * Near/far correction for this distance (robot center to cell): NEAR_MULTIPLIER closer
     * than the split, FAR_MULTIPLIER farther, blended linearly across SPLIT_BLEND_IN.
     */
    public static double nearFarMultiplier(double distanceIn) {
        double range = distanceIn - RANGE_OFFSET_IN;
        double start = NEAR_FAR_SPLIT_IN - SPLIT_BLEND_IN / 2;
        double end = NEAR_FAR_SPLIT_IN + SPLIT_BLEND_IN / 2;
        if (range <= start) return NEAR_MULTIPLIER;
        if (range >= end) return FAR_MULTIPLIER;
        double t = (range - start) / (end - start);
        return NEAR_MULTIPLIER + t * (FAR_MULTIPLIER - NEAR_MULTIPLIER);
    }

    // True if the ball can't reach the cell height from this close at the launch angle.
    public static boolean tooClose(double distanceIn) {
        return Double.isNaN(exitSpeed(distanceIn - RANGE_OFFSET_IN));
    }

    /*
     * Exit speed (in/s) needed to hit the target height at this horizontal range, from
     * y = x tan(a) - g x^2 / (2 v^2 cos^2(a)). Returns NaN when the shot is impossible.
     */
    private static double exitSpeed(double rangeIn) {
        double angle = Math.toRadians(LAUNCH_ANGLE_DEG);
        double rise = TARGET_HEIGHT_IN - LAUNCH_HEIGHT_IN;
        double cos = Math.cos(angle);
        double denominator = 2 * cos * cos * (rangeIn * Math.tan(angle) - rise);
        if (rangeIn <= 0 || denominator <= 0) {
            return Double.NaN;
        }
        return Math.sqrt(GRAVITY_IN_PER_S2 * rangeIn * rangeIn / denominator);
    }

    private FlywheelSpeedModel() {
    }
}
