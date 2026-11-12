package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;
import org.firstinspires.ftc.teamcode.Java.Field.PoseStorage;
import org.firstinspires.ftc.teamcode.Java.Shooter.FireFeedforward;
import org.firstinspires.ftc.teamcode.Java.Shooter.FlywheelSpeedModel;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/*
 * Pedro Pathing auto for LM1. RedAutoLM1 and BlueAutoLM1 only say which alliance they are.
 *
 * ROUTE (written for RED; Blue is the same route rotated 180 degrees around the field center):
 *   1. Start against the audience wall, back (shooter) toward the hive.
 *   2. Shoot the preloads FROM THE START SPOT (turns in place so the back points at the
 *      audience-side cell) at a fixed SHOT_VELOCITY of 1560 ticks/s, with the same fire
 *      feedforward as the teleop (balls 2-4 get boosted).
 *   3. Drive to the corner garden on the wall, front (intake) first, and intake for 2 s.
 *   4. Back up the red side (paths 3 and 4, shooter first), then drive to the final shot
 *      spot (path 5, intake first). The intake keeps running to hold the balls.
 *   5. Turn in place so the BACK (shooter) points straight at the far-side cell, and shoot.
 *
 * Positions are in the FieldLM1 frame (inches): x from the red wall (0) to the blue wall (144),
 * y from the audience side (0) to the far side (144), heading 0 = toward the blue wall, CCW
 * positive. ALL POSITIONS ARE FIRST GUESSES from the field drawing: test and adjust below.
 *
 * The robot's FRONT has the intake and its BACK has the shooter, so shots point the back at the
 * cell and intakes drive front first.
 */
public abstract class PedroAutoLM1 extends OpMode {

    protected abstract FieldLM1.Alliance alliance();

    // ---------- Route, RED side (x, y, heading in degrees) ----------
    /*
     * Route from the team's Pedro visualizer file (2026-11-11).
     * Start: front (intake) toward the audience wall, back (shooter) toward the hive, so the
     * heading is -90 (270). The visualizer file had 90, which would mean the INTAKE faces the
     * hive; -90 matches "shooter facing the goal".
     */
    public static double[] START = {56, 8, -90};
    // Path 1: to the garden approach, still facing the wall (front first).
    public static double[] GARDEN_APPROACH = {15.0395, 23.0705, -90};
    // Path 2: into the wall garden, front (intake) first. Visualizer "tangential" heading =
    // facing the direction of travel, which is -93.6 deg here.
    // 4 in deeper into the balls than the visualizer point (14.1896, 9.3797), same direction.
    public static double[] GARDEN_INTAKE = {13.9416, 5.3877, -93.5521};

    // Path 3: back out of the garden up the red side, shooter (back) first.
    public static double[] POINT_3 = {26.1485, 63.593, -102.4396};
    // Path 4: keep going up the red side, shooter first.
    public static double[] POINT_4 = {27.3506, 117.0332, -91.2886};
    // Path 5: across to the final shot spot, intake first.
    public static double[] FINAL_SHOT = {62.3445, 120.1809, 5.14};

    // Which cell each shot aims at (true = audience-side cell, false = far-side cell).
    public static boolean SHOT_AUDIENCE_CELL = true;
    public static boolean FINAL_SHOT_AUDIENCE_CELL = false;

    // ---------- Timing ----------
    public static double FEED_SECONDS = 3.5;          // how long to run intake + transfer per volley
    public static double MIN_SPIN_UP_SECONDS = 2.0;   // always wait at least this long before feeding
    public static double SHOOT_READY_TIMEOUT = 4.0;   // feed anyway if not aimed/at speed by then
    public static double GARDEN_INTAKE_SECONDS = 2.0; // extra intake time after reaching the garden
    public static double DRIVE_TIMEOUT = 4.0;         // give up on a drive after this long
    public static double AT_END_INCHES = 2.0;         // a drive is done this close to its end

    // ---------- Shooter / intake (same as the teleop) ----------
    public static double INTAKE_POWER = 1;
    public static double TRANSFER_FEED_POWER = 1;
    public static double INTAKE_FIRE_POWER = 1;
    public static double AIM_TOLERANCE_DEG = 3;
    public static double LAUNCHER_TOLERANCE = 50;
    // Fixed shot speed (ticks/s), same launcher PIDF (F = 14.4) and fire feedforward as the teleop.
    public static double SHOT_VELOCITY = 1560;

    private Follower follower;
    private DcMotorEx launcher;
    private DcMotor intake;
    private DcMotor transfer;

    private final List<Step> steps = new ArrayList<>();
    private int stepIndex = 0;
    private boolean stepStarted = false;

    private double flywheelTarget = SHOT_VELOCITY;
    private double intakePower = 0;
    private double transferPower = 0;
    // True while a Shoot step is feeding balls; FireFeedforward boosts the flywheel after ball 1.
    private boolean firing = false;
    private final FireFeedforward fireFeedforward = new FireFeedforward();
    private final ElapsedTime autoTimer = new ElapsedTime();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        // Recalibrate the Pinpoint's gyro while the robot sits still at INIT (prevents heading
        // drift). KEEP THE ROBOT STILL during INIT. The start position is set again at START.
        hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get()).recalibrateIMU();

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(40, 0, 0, 14.4));

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(BRAKE);
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setZeroPowerBehavior(BRAKE);

        buildRoute();
        follower.setPose(pose(START));
    }

    @Override
    public void init_loop() {
        Pose s = pose(START);
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Place robot at", "x %.0f, y %.0f, heading %.0f deg (back toward the hive)",
                s.x(), s.y(), Math.toDegrees(s.heading()));
        telemetry.update();
    }

    @Override
    public void start() {
        // Set again at START in case the Pinpoint was still resetting during INIT.
        follower.setPose(pose(START));
        autoTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        intakePower = 0;
        transferPower = 0;
        firing = false;

        String stepName = "DONE";
        if (stepIndex < steps.size()) {
            Step step = steps.get(stepIndex);
            stepName = step.name;
            if (!stepStarted) {
                step.begin();
                stepStarted = true;
            }
            if (step.run()) {
                stepIndex++;
                stepStarted = false;
            }
        }

        launcher.setVelocity(fireFeedforward.command((int) Math.round(flywheelTarget), launcher.getVelocity(), firing));
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        // Hand the position to RedTeleopLM1 / BlueTeleopLM1.
        Pose p = follower.pose();
        PoseStorage.save(p, alliance());

        telemetry.addData("Step", "%d/%d  %s", Math.min(stepIndex + 1, steps.size()), steps.size(), stepName);
        telemetry.addData("Time", "%.1f s", autoTimer.seconds());
        telemetry.addData("Pose", "x %.1f, y %.1f, heading %.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addData("Flywheel", "target %.0f, actual %.0f ticks/s", flywheelTarget, launcher.getVelocity());
        telemetry.update();
    }

    @Override
    public void stop() {
        PoseStorage.save(follower.pose(), alliance());
        follower.stop();
        launcher.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0);
    }

    // ---------- Route ----------

    private void buildRoute() {
        Pose cell = cell(SHOT_AUDIENCE_CELL);

        steps.add(new Shoot("Shoot preloads from start", cell));
        steps.add(new Drive("Drive to garden", pose(GARDEN_APPROACH), null, true, null));
        steps.add(new Drive("Intake garden", pose(GARDEN_INTAKE), null, true, null));
        steps.add(new Wait("Intake garden (wait)", GARDEN_INTAKE_SECONDS, true));

        Pose finalCell = cell(FINAL_SHOT_AUDIENCE_CELL);
        steps.add(new Drive("Path 3 (back up the side)", pose(POINT_3), null, true, null));
        steps.add(new Drive("Path 4 (up the side)", pose(POINT_4), null, true, null));
        steps.add(new Drive("Path 5 (to final shot)", pose(FINAL_SHOT), null, true, null));
        // Shoot turns in place so the back points straight at the cell, then fires.
        steps.add(new Shoot("Final shot", finalCell));
    }

    // ---------- Steps ----------

    private abstract static class Step {
        final String name;

        Step(String name) {
            this.name = name;
        }

        void begin() {
        }

        // Returns true when the step is finished.
        abstract boolean run();
    }

    /*
     * Drive along a path to `end` (a straight line, or a curve bending toward `bend`), turning
     * smoothly to end.heading(). Optionally runs the intake, and pre-spins the flywheel for the
     * shot at `spinFor` so it's ready on arrival.
     */
    private class Drive extends Step {
        final Pose end;
        final Pose bend;
        final boolean runIntake;
        final Pose spinFor;
        final ElapsedTime timer = new ElapsedTime();

        Drive(String name, Pose end, Pose bend, boolean runIntake, Pose spinFor) {
            super(name);
            this.end = end;
            this.bend = bend;
            this.runIntake = runIntake;
            this.spinFor = spinFor;
        }

        @Override
        void begin() {
            Pose start = follower.pose();
            // Turn the short way to the end heading.
            double endHeading = start.heading() + wrap(end.heading() - start.heading());
            Path path = (bend == null) ? line(start, end) : curve(start, bend, end);
            follower.follow(path.linear(start.heading(), endHeading));
            if (spinFor != null) {
                flywheelTarget = FlywheelSpeedModel.velocityForDistance(end.distance(spinFor));
            }
            timer.reset();
        }

        @Override
        boolean run() {
            if (runIntake) {
                intakePower = INTAKE_POWER;
            }
            boolean arrived = follower.atParametricEnd() && follower.distanceToEndpoint() < AT_END_INCHES;
            if (arrived || timer.seconds() > DRIVE_TIMEOUT) {
                follower.hold(end);
                return true;
            }
            return false;
        }
    }

    // Hold position with the back pointed at the cell, wait for the flywheel, then feed.
    private class Shoot extends Step {
        final Pose cell;
        final ElapsedTime readyTimer = new ElapsedTime();
        final ElapsedTime feedTimer = new ElapsedTime();
        double aimHeading;
        boolean feeding;

        Shoot(String name, Pose cell) {
            super(name);
            this.cell = cell;
        }

        @Override
        void begin() {
            Pose p = follower.pose();
            Pose aimed = aimPose(p, cell);
            aimHeading = aimed.heading();
            follower.hold(aimed);
            flywheelTarget = SHOT_VELOCITY;
            feeding = false;
            readyTimer.reset();
        }

        @Override
        boolean run() {
            if (!feeding) {
                boolean aimed = Math.abs(Math.toDegrees(wrap(aimHeading - follower.pose().heading()))) <= AIM_TOLERANCE_DEG;
                boolean atSpeed = Math.abs(launcher.getVelocity() - flywheelTarget) <= LAUNCHER_TOLERANCE;
                boolean spunUp = readyTimer.seconds() > MIN_SPIN_UP_SECONDS;
                if ((aimed && atSpeed && spunUp) || readyTimer.seconds() > SHOOT_READY_TIMEOUT) {
                    feeding = true;
                    feedTimer.reset();
                }
                return false;
            }
            transferPower = TRANSFER_FEED_POWER;
            intakePower = INTAKE_FIRE_POWER;
            firing = true;
            return feedTimer.seconds() > FEED_SECONDS;
        }
    }

    // Stay put for a while, optionally running the intake.
    private class Wait extends Step {
        final double seconds;
        final boolean runIntake;
        final ElapsedTime timer = new ElapsedTime();

        Wait(String name, double seconds, boolean runIntake) {
            super(name);
            this.seconds = seconds;
            this.runIntake = runIntake;
        }

        @Override
        void begin() {
            timer.reset();
        }

        @Override
        boolean run() {
            if (runIntake) {
                intakePower = INTAKE_POWER;
            }
            return timer.seconds() > seconds;
        }
    }

    // ---------- Field helpers ----------

    // A route pose (red side) on this alliance's side of the field.
    private Pose pose(double[] xyHeadingDeg) {
        return toAlliance(new Pose(xyHeadingDeg[0], xyHeadingDeg[1], Math.toRadians(xyHeadingDeg[2])));
    }

    // A route point (red side, no heading) on this alliance's side of the field.
    private Pose point(double[] xy) {
        return toAlliance(new Pose(xy[0], xy[1], 0));
    }

    /*
     * The real (measured) cell that matches red's audience-side or far-side cell on this
     * alliance's side. The route is rotated 180 deg for blue, so red's audience-side cell
     * becomes blue's FAR-side cell and vice versa.
     */
    private Pose cell(boolean redAudienceSide) {
        if (alliance() == FieldLM1.Alliance.RED) {
            return redAudienceSide ? FieldLM1.audienceCell(FieldLM1.Alliance.RED) : FieldLM1.farCell(FieldLM1.Alliance.RED);
        }
        return redAudienceSide ? FieldLM1.farCell(FieldLM1.Alliance.BLUE) : FieldLM1.audienceCell(FieldLM1.Alliance.BLUE);
    }

    // Blue = red rotated 180 degrees around the field center.
    private Pose toAlliance(Pose red) {
        if (alliance() == FieldLM1.Alliance.RED) {
            return red;
        }
        double size = FieldLM1.FIELD_SIZE;
        return new Pose(size - red.x(), size - red.y(), wrap(red.heading() + Math.PI));
    }

    // Same spot, heading turned so the BACK (shooter) points at the cell.
    private static Pose aimPose(Pose spot, Pose cell) {
        double faceCell = Math.atan2(cell.y() - spot.y(), cell.x() - spot.x());
        return new Pose(spot.x(), spot.y(), wrap(faceCell + Math.PI));
    }

    // Keep an angle between -180 and 180 degrees (in radians).
    private static double wrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
