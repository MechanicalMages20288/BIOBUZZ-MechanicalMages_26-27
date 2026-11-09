package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Simple auto: drives straight 12 inches, then holds still. Works for either alliance.
 *
 * The robot starts with its SHOOTER (back) facing the goal, so by default it drives
 * BACKWARD, shooter first, toward the goal. Set DISTANCE_IN positive to drive forward
 * (intake first, away from the goal) instead.
 *
 * Uses Pedro Pathing, so the distance is measured by the Pinpoint (not timed).
 */
@Autonomous(name = "SimpleMoveAutoLM1", group = "LM1")
public class SimpleMoveAutoLM1 extends OpMode {

    // Negative = backward (shooter first, toward the goal). Positive = forward.
    public static double DISTANCE_IN = -12;

    // Stop trying after this long, in case the robot is blocked.
    public static double TIMEOUT_SECONDS = 4;

    // Counts as arrived this close to the end (inches).
    public static double AT_END_INCHES = 1.0;

    private Follower follower;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean done = false;
    private Pose end;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        // Recalibrate the gyro while the robot is still. KEEP THE ROBOT STILL during INIT.
        hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get()).recalibrateIMU();
        telemetry.addData("Will drive", "%.0f in %s", Math.abs(DISTANCE_IN),
                DISTANCE_IN < 0 ? "BACKWARD (shooter first, toward the goal)" : "FORWARD (intake first)");
    }

    @Override
    public void start() {
        // Measured from wherever the robot starts: heading 0 = the way the robot's front faces.
        Pose start = new Pose(0, 0, 0);
        end = new Pose(DISTANCE_IN, 0, 0);
        follower.setPose(start);
        follower.follow(line(start, end).linear(0, 0));
        timer.reset();
    }

    @Override
    public void loop() {
        follower.update();

        if (!done) {
            boolean arrived = follower.atParametricEnd() && follower.distanceToEndpoint() < AT_END_INCHES;
            if (arrived || timer.seconds() > TIMEOUT_SECONDS) {
                follower.hold(end);
                done = true;
            }
        }

        Pose p = follower.pose();
        telemetry.addData("State", done ? "DONE (holding)" : "DRIVING");
        telemetry.addData("Moved", "%.1f of %.0f in", p.x(), DISTANCE_IN);
        telemetry.addData("Pose", "x %.1f, y %.1f, heading %.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.stop();
    }
}
