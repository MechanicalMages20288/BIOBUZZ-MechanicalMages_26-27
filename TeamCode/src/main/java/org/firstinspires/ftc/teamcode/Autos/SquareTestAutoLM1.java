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
 * Square test: drives a SIDE_IN x SIDE_IN square and ends where it started.
 * The robot never turns: it keeps facing the same way the whole time, so two sides are
 * driven forward/backward and two sides are STRAFED.
 *
 *   1. forward SIDE_IN
 *   2. strafe left SIDE_IN
 *   3. backward SIDE_IN
 *   4. strafe right SIDE_IN (back to the start)
 *
 * Uses Pedro Pathing, so distances come from the Pinpoint. Start anywhere with some room.
 */
@Autonomous(name = "SquareTestAutoLM1", group = "LM1")
public class SquareTestAutoLM1 extends OpMode {

    // Length of each side of the square (inches).
    public static double SIDE_IN = 5;

    // Give up on a side after this long, in case the robot is blocked.
    public static double SIDE_TIMEOUT_SECONDS = 3;

    // A side counts as done this close to its corner (inches).
    public static double AT_END_INCHES = 0.5;

    private Follower follower;
    private Pose[] corners;
    private final String[] names = {"forward", "strafe left", "backward", "strafe right"};
    private int side = 0;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        // Recalibrate the gyro while the robot is still. KEEP THE ROBOT STILL during INIT.
        hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get()).recalibrateIMU();
        telemetry.addData("Will drive", "a %.0f in square, no turning", SIDE_IN);
    }

    @Override
    public void start() {
        // Heading 0 = the way the robot faces at the start. +x = forward, +y = left.
        corners = new Pose[]{
                new Pose(0, 0, 0),
                new Pose(SIDE_IN, 0, 0),
                new Pose(SIDE_IN, SIDE_IN, 0),
                new Pose(0, SIDE_IN, 0),
                new Pose(0, 0, 0),
        };
        follower.setPose(corners[0]);
        followSide();
    }

    private void followSide() {
        // Constant heading 0 on every side, so the robot strafes instead of turning.
        follower.follow(line(corners[side], corners[side + 1]).linear(0, 0));
        timer.reset();
    }

    @Override
    public void loop() {
        follower.update();

        if (side < 4) {
            boolean arrived = follower.atParametricEnd() && follower.distanceToEndpoint() < AT_END_INCHES;
            if (arrived || timer.seconds() > SIDE_TIMEOUT_SECONDS) {
                side++;
                if (side < 4) {
                    followSide();
                } else {
                    follower.hold(corners[4]);
                }
            }
        }

        Pose p = follower.pose();
        telemetry.addData("Side", side < 4 ? String.format("%d/4  %s", side + 1, names[side]) : "DONE (back at start, holding)");
        telemetry.addData("Pose", "x %.2f, y %.2f, heading %.1f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        follower.stop();
    }
}
