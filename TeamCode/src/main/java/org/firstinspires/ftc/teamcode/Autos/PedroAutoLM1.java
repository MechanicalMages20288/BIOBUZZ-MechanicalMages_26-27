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
 *   2. Drive to SHOT_1 and shoot the preloads into the audience-side cell.
 *   3. Drive to the corner garden, front (intake) first, and intake.
 *   4. Drive around the left of the hive to SHOT_2 and shoot into the far-side cell.
 *   5. Drive to the top-wall flower, front first, and intake.
 *   6. Back to SHOT_3 and shoot into the far-side cell again.
 *   7. Park in the loading zone.
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
    // Start: front against the audience wall, square to it, back toward the hive.
    // (Robot center is 12 in from its front edge, so against a wall the center is 12 in off it.)
    public static double[] START = {44, 12, -90};
    // Shot spots (x, y). Heading is worked out so the back points at the target cell.
    public static double[] SHOT_1 = {46, 34};
    public static double[] SHOT_2 = {52, 112};
    public static double[] SHOT_3 = {52, 112};
    // Corner garden: approach, then drive in front first to intake.
    public static double[] GARDEN_APPROACH = {16, 24, -90};
    public static double[] GARDEN_INTAKE = {14, 13, -90};
    // Bend point so the drive to SHOT_2 goes around the left side of the hive.
    public static double[] AROUND_HIVE = {20, 80};
    // Top-wall flower: approach, then drive in front first to intake.
    public static double[] FLOWER_APPROACH = {47.6, 116, 90};
    public static double[] FLOWER_INTAKE = {47.6, 128, 90};
    // Park: front into the loading zone on the red wall.
    public static double[] PARK = {13, 108, 180};

    // Which cell each shot aims at (true = audience-side cell, false = far-side cell).
    public static boolean SHOT_1_AUDIENCE_CELL = true;
    public static boolean SHOT_2_AUDIENCE_CELL = false;
    public static boolean SHOT_3_AUDIENCE_CELL = false;

    // ---------- Timing ----------
    public static double FEED_SECONDS = 2.5;          // how long to run intake + transfer per volley
    public static double SHOOT_READY_TIMEOUT = 2.0;   // feed anyway if not aimed/at speed by then
    public static double GARDEN_INTAKE_SECONDS = 1.0; // extra intake time after reaching the garden
    public static double FLOWER_INTAKE_SECONDS = 1.5; // extra intake time at the flower
    public static double DRIVE_TIMEOUT = 4.0;         // give up on a drive after this long
    public static double AT_END_INCHES = 2.0;         // a drive is done this close to its end

    // ---------- Shooter / intake (same as the teleop) ----------
    public static double INTAKE_POWER = 1;
    public static double TRANSFER_FEED_POWER = 1;
    public static double INTAKE_FIRE_POWER = 1;
    public static double AIM_TOLERANCE_DEG = 3;
    public static double LAUNCHER_TOLERANCE = 50;
    public static double IDLE_FLYWHEEL_VELOCITY = 1600;

    private Follower follower;
    private DcMotorEx launcher;
    private DcMotor intake;
    private DcMotor transfer;

    private final List<Step> steps = new ArrayList<>();
    private int stepIndex = 0;
    private boolean stepStarted = false;

    private double flywheelTarget = IDLE_FLYWHEEL_VELOCITY;
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
        Pose cell1 = cell(SHOT_1_AUDIENCE_CELL);
        Pose cell2 = cell(SHOT_2_AUDIENCE_CELL);
        Pose cell3 = cell(SHOT_3_AUDIENCE_CELL);
        Pose shot1 = aimPose(point(SHOT_1), cell1);
        Pose shot2 = aimPose(point(SHOT_2), cell2);
        Pose shot3 = aimPose(point(SHOT_3), cell3);

        steps.add(new Drive("Drive to shot 1", shot1, null, false, cell1));
        steps.add(new Shoot("Shoot preloads", cell1));
        steps.add(new Drive("Drive to garden", pose(GARDEN_APPROACH), null, true, null));
        steps.add(new Drive("Intake garden", pose(GARDEN_INTAKE), null, true, null));
        steps.add(new Wait("Intake garden (wait)", GARDEN_INTAKE_SECONDS, true));
        steps.add(new Drive("Drive to shot 2", shot2, point(AROUND_HIVE), true, cell2));
        steps.add(new Shoot("Shoot garden balls", cell2));
        steps.add(new Drive("Drive to flower", pose(FLOWER_APPROACH), null, true, null));
        steps.add(new Drive("Intake flower", pose(FLOWER_INTAKE), null, true, null));
        steps.add(new Wait("Intake flower (wait)", FLOWER_INTAKE_SECONDS, true));
        steps.add(new Drive("Drive to shot 3", shot3, null, false, cell3));
        steps.add(new Shoot("Shoot flower balls", cell3));
        steps.add(new Drive("Park", pose(PARK), null, false, null));
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
            flywheelTarget = FlywheelSpeedModel.velocityForDistance(p.distance(cell));
            feeding = false;
            readyTimer.reset();
        }

        @Override
        boolean run() {
            if (!feeding) {
                boolean aimed = Math.abs(Math.toDegrees(wrap(aimHeading - follower.pose().heading()))) <= AIM_TOLERANCE_DEG;
                boolean atSpeed = Math.abs(launcher.getVelocity() - flywheelTarget) <= LAUNCHER_TOLERANCE;
                if ((aimed && atSpeed) || readyTimer.seconds() > SHOOT_READY_TIMEOUT) {
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
