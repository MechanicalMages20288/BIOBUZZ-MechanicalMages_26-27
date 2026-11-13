package org.firstinspires.ftc.teamcode.Java.Shooter;

/*
 * Firing feedforward: keeps the flywheel speed up through a 4-ball burst.
 *
 * THE PROBLEM: each ball steals speed from the flywheel. The first ball leaves at the right
 * speed, but the wheel can't recover before the next ball arrives, so balls 2-4 go short.
 *
 * THE FIX: the base speed stays where ONE ball lands right (e.g. 1500). As soon as the first
 * ball is detected leaving (the wheel speed drops by SHOT_DETECT_DROP), the launcher target is
 * raised for the rest of the burst:
 *     boost = FIRE_FEEDFORWARD + RECOVERY_GAIN x (how far the wheel is below its pre-fire speed)
 * The fixed part pushes extra power in ahead of the next balls; the recovery part pushes harder
 * the more speed a ball took. Ball 1 is never boosted, so it still lands where it did before.
 * The boost resets when you stop firing.
 *
 * TUNING (fire 4 balls from ~29 in and watch where 2-4 land compared to ball 1):
 *   Balls 2-4 still short              -> raise FIRE_FEEDFORWARD (+50 at a time)
 *   Balls 2-4 long                     -> lower FIRE_FEEDFORWARD
 *   Ball 2 short but ball 4 long       -> raise RECOVERY_GAIN, lower FIRE_FEEDFORWARD
 *   Boost never turns on (telemetry)   -> lower SHOT_DETECT_DROP
 *   Boost turns on before ball 1 leaves -> raise SHOT_DETECT_DROP
 */
public class FireFeedforward {

    // Extra target speed (ticks/s) for the rest of the burst once the first ball has left.
    public static double FIRE_FEEDFORWARD = 200;

    // Extra target speed per tick/s the wheel is below its pre-fire speed.
    public static double RECOVERY_GAIN = 2.0;

    // A speed drop this big (ticks/s) while firing counts as "ball 1 has left".
    public static double SHOT_DETECT_DROP = 60;

    // Never boost the target by more than this.
    public static double MAX_BOOST = 900;

    /*
     * Boost only during the first part of each shooting sequence: balls 1-3 go back to back and
     * need it, but ball 4 lags behind and would go long if the wheel were still boosted.
     * Measured from when firing starts (trigger pulled / auto starts feeding).
     */
    public static double BOOST_WINDOW_SECONDS = 0.5;

    // How quickly the pre-fire speed follows the real speed while not firing (0-1 per loop).
    public static double READY_SPEED_FILTER = 0.3;

    // This instance's fixed boost (the auto can use a different one than the teleop).
    private final double feedforward;

    // Uses FIRE_FEEDFORWARD (the teleop value).
    public FireFeedforward() {
        this(FIRE_FEEDFORWARD);
    }

    public FireFeedforward(double feedforward) {
        this.feedforward = feedforward;
    }

    private double readySpeed = Double.NaN;
    private boolean firstShotSeen = false;
    private double boost = 0;
    private double lowestThisBurst = Double.NaN;
    private double lowestLastBurst = Double.NaN;
    private boolean wasFiring = false;
    private long firingStartNs = 0;
    private boolean windowOver = false;

    /*
     * Call once per loop. Returns the target to send to launcher.setVelocity().
     *   baseTarget  the normal speed for this shot (1500, auto speed, etc.)
     *   actual      launcher.getVelocity()
     *   firing      true while balls are being fed in
     */
    public int command(int baseTarget, double actual, boolean firing) {
        if (!firing) {
            if (wasFiring) {
                lowestLastBurst = lowestThisBurst;
            }
            readySpeed = Double.isNaN(readySpeed) ? actual
                    : readySpeed + READY_SPEED_FILTER * (actual - readySpeed);
            firstShotSeen = false;
            boost = 0;
            wasFiring = false;
            windowOver = false;
            return baseTarget;
        }

        if (!wasFiring) {
            firingStartNs = System.nanoTime();
            lowestThisBurst = actual;
            if (Double.isNaN(readySpeed)) readySpeed = actual;
        }
        wasFiring = true;
        lowestThisBurst = Math.min(lowestThisBurst, actual);

        double drop = readySpeed - actual;
        if (drop > SHOT_DETECT_DROP) {
            firstShotSeen = true;
        }
        windowOver = (System.nanoTime() - firingStartNs) / 1e9 > BOOST_WINDOW_SECONDS;
        boost = (firstShotSeen && !windowOver)
                ? Math.max(0, Math.min(MAX_BOOST, feedforward + RECOVERY_GAIN * drop))
                : 0;
        return (int) Math.round(baseTarget + boost);
    }

    // Current boost on top of the base target (0 when not boosting).
    public double boost() {
        return boost;
    }

    public boolean boosting() {
        return firstShotSeen && !windowOver;
    }

    // Wheel speed just before the current/last burst started.
    public double readySpeed() {
        return readySpeed;
    }

    // Lowest wheel speed during the last finished burst (NaN before the first burst).
    public double lowestLastBurst() {
        return lowestLastBurst;
    }
}
