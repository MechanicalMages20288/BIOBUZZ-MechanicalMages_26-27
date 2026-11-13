package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;
import org.firstinspires.ftc.teamcode.Java.Field.PoseStorage;
import org.firstinspires.ftc.teamcode.Java.Shooter.FireFeedforward;
import org.firstinspires.ftc.teamcode.Java.Shooter.FlywheelSpeedModel;
import org.firstinspires.ftc.teamcode.Java.Teleop.BasicTeleopLM1;
import org.firstinspires.ftc.teamcode.Java.Teleop.LimelightPositionLM1;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.Java.Shooter.LauncherVoltageComp;

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
 *   3. Drive into the corner (side close to the red wall, front into the audience wall),
 *      intake on, then back out and push in again INTAKE_PASSES times to grab wall balls.
 *   4. Back up the red side (paths 3 and 4, shooter first), then drive to the final shot
 *      spot (path 5, intake first). The intake keeps running to hold the balls.
 *   5. Turn in place so the BACK (shooter) points straight at the far-side cell, and shoot.
 *   6. Flower: drive to FLOWER_APPROACH, lower the flower servo, drive forward to
 *      FLOWER_INTAKE while intaking, back up FLOWER_BACK_UP_IN, raise the flower servo,
 *      return to the final shot spot and shoot again.
 *   7. Park: drive straight into the park zone at PARK.
 *
 * TAG CHECK AFTER THE PRELOADS (only the Tag autos: Red/BlueTagAutoLM1 and
 * Red/BlueTag2xFlowerAutoLM1; useTagCheck() = true): right after the first shot, the robot stays aimed and the
 * Limelight watches the tags of the cell it just shot into (red audience-side cell 34-37,
 * blue far-side cell 42-45). The camera faces backward, the same way as the shooter.
 *   - Tag gone (not seen for TAG_LOST_GRACE_SECONDS): leave right away on the route above.
 *   - Tag still visible for the whole TAG_WATCH_SECONDS (3 s): that cell is still facing us,
 *     so the route changes to: intake (step 3) -> back to START -> shoot that cell again ->
 *     up the red side to the top flower -> final shot spot, shoot the far-side cell -> park.
 *     Before the second start shot and before the top flower trip it checks the clock; if
 *     there isn't enough time left (RETURN_SHOT_SECONDS_NEEDED / FLOWER_TRIP_SECONDS_NEEDED)
 *     it skips straight to parking.
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
    public static double[] START = {58, 12, -90};
    /*
     * Corner intake. x = 10 pushes the robot's side about 2 in into the red wall (robot is 24 in
     * wide). Pushes go to y = 8, a few inches past where the front meets the audience wall
     * (center y = 12), so the intake presses into the balls; each push gives up after
     * PUSH_TIMEOUT once the wall stops it.
     */
    // Path 1: line up above the corner, facing the audience wall (front first).
    public static double[] GARDEN_APPROACH = {8, 24, -90};
    // Path 2: push into the corner, front (intake) first.
    public static double[] GARDEN_INTAKE = {8, 8, -90};
    // Back out to here between pushes, then push in again.
    public static double[] GARDEN_BACK_OUT = {10, 20, -90};
    public static int INTAKE_PASSES = 2;          // total pushes into the corner (1 + 2 more)
    public static double PUSH_TIMEOUT = 0.85;     // seconds per push / back-out

    // Path 3: back out of the garden up the red side, shooter (back) first.
    public static double[] POINT_3 = {26.1485, 63.593, -102.4396};
    // Path 4: keep going up the red side, shooter first.
    public static double[] POINT_4 = {27.3506, 117.0332, -91.2886};
    // Path 5: to the final shot spot = the START mirrored to the far side (y -> 144 - y, heading
    // flipped): front toward the far wall, shooter toward the hive.
    // 1 in closer to the far-side red cell than (59, 136), along the line to the cell.
    public static double[] FINAL_SHOT = {58.97, 135.00, 90};
    // Path 5 curves through here: dips right/down from point 4, then swings up into the shot
    // spot (the team's drawn route). Pedro curve control point (x, y).
    public static double[] PATH_5_BEND = {55, 105};

    // Flower (top wall), from the team's visualizer file. Front (intake) toward the flower.
    public static double[] FLOWER_APPROACH = {46.5874, 109.8841, 90};
    // 9.5 in further into the flower than the visualizer point (y 121.231).
    // Normal-speed drive in stops here (1 in short of the old 130.731)...
    public static double[] FLOWER_INTAKE = {46.5873, 129.731, 90};
    // ...then a slow push the rest of the way into the flower.
    /*
     * Alliance (side-wall) flower, used by the 2x Flower autos INSTEAD of the corner wall
     * intake. The red alliance flower is on the red wall (x = 0) at about y = 46. Same spacing
     * from the wall as the top flower: approach 31.62 in off the wall, normal-speed drive in to
     * 11.77 in, then the same slow push / wait / back-out / lift.
     */
    // y 48: 2 in toward the far side from the original 46.
    public static double[] SIDE_FLOWER_APPROACH = {31.62, 48, 180};
    public static double[] SIDE_FLOWER_INTAKE = {13, 48, 180};
    // Side flower backs out 2 in more than the top flower before lifting the servo.
    public static double SIDE_FLOWER_BACK_UP_IN = 8;

    public static double FLOWER_SLOW_PUSH_IN = 2;        // inches
    public static double FLOWER_SLOW_POWER = 0.3;        // drive power (0-1)
    public static double FLOWER_SLOW_TIMEOUT = 1.5;      // give up if the flower stops it
    public static double FLOWER_INTAKE_WAIT_SECONDS = 1.75; // stay at the flower, intaking
    public static double FLOWER_BACK_UP_IN = 6;        // back up this far BEFORE raising the servo
    public static double FLOWER_SERVO_SECONDS = 0.2;   // time for the servo to move (raising)
    public static double FLOWER_LOWER_SECONDS = 1.0;   // after lowering: wait for the attachment to tip

    // Park spot after the last shot (team's point). Heading 90 = same way the robot already
    // faces after shooting, so it just slides over without turning.
    public static double[] PARK = {10, 122, 90};

    // Which cell each shot aims at (true = audience-side cell, false = far-side cell).
    public static boolean SHOT_AUDIENCE_CELL = true;
    public static boolean FINAL_SHOT_AUDIENCE_CELL = false;

    // ---------- Timing ----------
    public static double FEED_SECONDS = 3;          // how long to run intake + transfer per volley
    public static double MIN_SPIN_UP_SECONDS = 2.0;
    // Shot 2 (first far-side shot): flywheel has been spinning the whole drive, so 1 s less.
    public static double SHOT_2_MIN_SPIN_UP_SECONDS = 1.0;   // always wait at least this long before feeding
    public static double SHOOT_READY_TIMEOUT = 4.0;   // feed anyway if not aimed/at speed by then
    public static double GARDEN_INTAKE_SECONDS = 0.0; // extra intake time after the last push
    public static double DRIVE_TIMEOUT = 4.0;         // give up on a drive after this long
    public static double AT_END_INCHES = 2.0;         // a drive is done this close to its end

    // ---------- Tag check after the preloads ----------
    // Tags of the cell the preloads go into (game manual 9.9).
    public static int RED_PRELOAD_CELL_FIRST_TAG = 34, RED_PRELOAD_CELL_LAST_TAG = 37; // red audience-side
    public static int BLUE_PRELOAD_CELL_FIRST_TAG = 42, BLUE_PRELOAD_CELL_LAST_TAG = 45; // blue far-side
    // The tag must stay visible this long to take the "shoot that cell again" route...
    public static double TAG_WATCH_SECONDS = 3.0;
    // ...and counts as gone once it hasn't been seen for this long (rides out dropped frames).
    public static double TAG_LOST_GRACE_SECONDS = 0.3;
    public static double TAG_MAX_STALENESS_MS = 200;  // ignore older Limelight frames

    // Time checks in the "shoot again" route (seconds that must be LEFT in the 30 s auto,
    // including time to park afterward). Not enough -> skip straight to parking.
    public static double AUTO_SECONDS = 30;
    public static double RETURN_SHOT_SECONDS_NEEDED = 13;  // drive back to start + shoot + park
    public static double FLOWER_TRIP_SECONDS_NEEDED = 18;  // up the side + top flower + far shot + park

    // ---------- Shooter / intake (same as the teleop) ----------
    public static double INTAKE_POWER = 1;
    public static double TRANSFER_FEED_POWER = 1;
    // When NOT shooting, the transfer turns slowly BACKWARD so balls can't slip through to the
    // flywheel. Negative = backward. Shooting switches it to full forward (TRANSFER_FEED_POWER).
    public static double TRANSFER_HOLD_POWER = -0.1;
    public static double INTAKE_FIRE_POWER = 1;
    public static double AIM_TOLERANCE_DEG = 3;
    public static double LAUNCHER_TOLERANCE = 50;
    // Fixed shot speed (ticks/s), same launcher PIDF (F = 14.4) and fire feedforward as the teleop.
    public static double SHOT_VELOCITY = 1530;

    private Follower follower;
    private Limelight3A limelight;
    private String routeNote = "watching preload cell tags";
    private DcMotorEx launcher;
    private LauncherVoltageComp launcherVoltage;
    private DcMotor intake;
    private DcMotor transfer;
    private Servo flowerServo;

    private final List<Step> steps = new ArrayList<>();
    private int stepIndex = 0;
    private boolean stepStarted = false;

    private double flywheelTarget = SHOT_VELOCITY;
    private double intakePower = 0;
    private double transferPower = 0;
    // True while a Shoot step is feeding balls; FireFeedforward boosts the flywheel after ball 1.
    private boolean firing = false;
    // Auto's own firing feedforward, a bit stronger than the teleop's (FireFeedforward.FIRE_FEEDFORWARD = 200).
    public static double AUTO_FIRE_FEEDFORWARD = 290;
    private final FireFeedforward fireFeedforward = new FireFeedforward(AUTO_FIRE_FEEDFORWARD);
    // First shot (preloads from the start) gets a stronger boost.
    public static double FIRST_SHOT_FIRE_FEEDFORWARD = 390;
    private final FireFeedforward firstShotFeedforward = new FireFeedforward(FIRST_SHOT_FIRE_FEEDFORWARD);
    private final ElapsedTime autoTimer = new ElapsedTime();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        // Recalibrate the Pinpoint's gyro while the robot sits still at INIT (prevents heading
        // drift). KEEP THE ROBOT STILL during INIT. The start position is set again at START.
        hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get()).recalibrateIMU();

        // Limelight for the tag check after the preloads (same pipeline as the teleops).
        // Only the Tag autos use it; the others don't touch the Limelight at all.
        if (useTagCheck()) {
            limelight = hardwareMap.get(Limelight3A.class, LimelightPositionLM1.LIMELIGHT_NAME);
            if (LimelightPositionLM1.UPLOAD_PIPELINE_AT_INIT) {
                limelight.uploadPipeline(LimelightPositionLM1.PIPELINE_JSON, LimelightPositionLM1.APRILTAG_PIPELINE);
            }
            limelight.pipelineSwitch(LimelightPositionLM1.APRILTAG_PIPELINE);
            limelight.setPollRateHz(100);
            limelight.start();
        }

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Launcher PIDF (40, 0, 0, 14.4) with battery voltage compensation on F.
        launcherVoltage = new LauncherVoltageComp(hardwareMap, launcher);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(BRAKE);
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        // Flower servo: same name and up/down positions as the teleop.
        flowerServo = hardwareMap.get(Servo.class, BasicTeleopLM1.FLOWER_SERVO_NAME);
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
        // Servo doesn't move until START.
        flowerServo.setPosition(BasicTeleopLM1.FLOWER_UP_POSITION);
    }

    @Override
    public void loop() {
        follower.update();
        intakePower = 0;
        // Slow backward hold only after the first shot (step 0); off before/during it.
        transferPower = stepIndex == 0 ? 0 : TRANSFER_HOLD_POWER;
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

        FireFeedforward ff = stepIndex == 0 ? firstShotFeedforward : fireFeedforward;
        launcherVoltage.update();
        launcher.setVelocity(ff.command((int) Math.round(flywheelTarget), launcher.getVelocity(), firing));
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        // Hand the position to RedTeleopLM1 / BlueTeleopLM1.
        Pose p = follower.pose();
        PoseStorage.save(p, alliance());

        telemetry.addData("Step", "%d/%d  %s", Math.min(stepIndex + 1, steps.size()), steps.size(), stepName);
        telemetry.addData("Time", "%.1f s", autoTimer.seconds());
        telemetry.addData("Route", routeNote);
        telemetry.addData("Pose", "x %.1f, y %.1f, heading %.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addData("Flywheel", "target %.0f, actual %.0f ticks/s", flywheelTarget, launcher.getVelocity());
        telemetry.addData("Battery", "%.2f V  (launcher F %.2f)", launcherVoltage.voltage(), launcherVoltage.f());
        telemetry.update();
    }

    @Override
    public void stop() {
        PoseStorage.save(follower.pose(), alliance());
        follower.stop();
        if (limelight != null) {
            limelight.stop();
        }
        launcher.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0);
    }

    // ---------- Route ----------

    /*
     * false (RedAutoLM1 / BlueAutoLM1): corner wall intake after the first shot.
     * true (Red/BlueLM12xFlowerAuto): the alliance side flower instead of the wall intake.
     */
    protected boolean sideFlowerInsteadOfWall() {
        return false;
    }

    /*
     * Only the preload shot and the tag check are added up front. The tag check then adds the
     * rest of the route: addNormalRoute() (tag gone) or addShootAgainRoute() (tag still there).
     */
    private void buildRoute() {
        steps.add(new Shoot("Shoot preloads from start", cell(SHOT_AUDIENCE_CELL)));
        if (useTagCheck()) {
            steps.add(new CheckPreloadCellTags("Watch preload cell tags"));
        } else {
            routeNote = "normal route (no tag check)";
            addNormalRoute();
        }
    }

    /*
     * false (RedAutoLM1, BlueAutoLM1, Red/BlueLM12xFlowerAuto): the original route, no tag check.
     * true (the Tag autos): watch the preload cell's tags after the first shot and pick the route.
     */
    protected boolean useTagCheck() {
        return false;
    }

    // Tag gone: the original route. Intake, up the side to the final shot, top flower, shoot, park.
    private void addNormalRoute() {
        addFirstIntake();

        Pose finalCell = cell(FINAL_SHOT_AUDIENCE_CELL);
        steps.add(new Drive("Path 3 (back up the side)", pose(POINT_3), null, false, null).passThrough());
        steps.add(new Drive("Path 4 (up the side)", pose(POINT_4), null, false, null));
        steps.add(new Drive("Path 5 (to final shot)", pose(FINAL_SHOT), point(PATH_5_BEND), false, null));
        // Shoot turns in place so the back points straight at the cell, then fires.
        steps.add(new Shoot("Final shot", finalCell, SHOT_2_MIN_SPIN_UP_SECONDS));

        // Top flower intake, then back to the same shot spot.
        addFlowerIntake("top flower", FLOWER_APPROACH, FLOWER_INTAKE, FLOWER_BACK_UP_IN);
        steps.add(new Drive("Back to shot spot", pose(FINAL_SHOT), null, false, null));
        steps.add(new Shoot("Flower shot", finalCell));

        // Park: straight into the park zone, keeping the same heading so there's no turn.
        steps.add(new Drive("Park", pose(PARK), null, false, null));
    }

    /*
     * Tag still visible: intake, back to START and shoot the same cell again, then up the side
     * to the top flower and shoot the far-side cell from the final shot spot, then park.
     * Each big chunk first checks there's enough time left; if not, it parks instead.
     */
    private void addShootAgainRoute() {
        addFirstIntake();
        steps.add(new Decide("Time check: shoot again from start?", () -> {
            if (timeLeft() < RETURN_SHOT_SECONDS_NEEDED) {
                routeNote += String.format(" | %.0f s left: skipped 2nd start shot, parking", timeLeft());
                addParkFromNearSide();
                return;
            }
            steps.add(new Drive("Back to start", pose(START), null, false, null));
            steps.add(new Shoot("Shoot same cell again", cell(SHOT_AUDIENCE_CELL), SHOT_2_MIN_SPIN_UP_SECONDS));
            steps.add(new Decide("Time check: top flower + far shot?", () -> {
                if (timeLeft() < FLOWER_TRIP_SECONDS_NEEDED) {
                    routeNote += String.format(" | %.0f s left: skipped top flower, parking", timeLeft());
                    addParkFromNearSide();
                    return;
                }
                Pose finalCell = cell(FINAL_SHOT_AUDIENCE_CELL);
                steps.add(new Drive("Path 3 (up the side)", pose(POINT_3), null, false, null).passThrough());
                steps.add(new Drive("Path 4 (up the side)", pose(POINT_4), null, false, null));
                addFlowerIntake("top flower", FLOWER_APPROACH, FLOWER_INTAKE, FLOWER_BACK_UP_IN);
                steps.add(new Drive("To final shot spot", pose(FINAL_SHOT), null, false, null));
                steps.add(new Shoot("Far cell shot", finalCell));
                steps.add(new Drive("Park", pose(PARK), null, false, null));
            }));
        }));
    }

    // Park from the audience half: up the red side (shooter first, no turning), then into PARK.
    private void addParkFromNearSide() {
        steps.add(new Drive("Park: up the side", pose(POINT_3), null, false, null).passThrough());
        steps.add(new Drive("Park: up the side", pose(POINT_4), null, false, null));
        steps.add(new Drive("Park", pose(new double[]{PARK[0], PARK[1], POINT_4[2]}), null, false, null));
    }

    private double timeLeft() {
        return AUTO_SECONDS - autoTimer.seconds();
    }

    // The first intake after the preloads: corner wall intake, or the side flower (2x Flower autos).
    private void addFirstIntake() {
        if (sideFlowerInsteadOfWall()) {
            addFlowerIntake("side flower", SIDE_FLOWER_APPROACH, SIDE_FLOWER_INTAKE, SIDE_FLOWER_BACK_UP_IN);
        } else {
            steps.add(new Drive("Drive to garden", pose(GARDEN_APPROACH), null, false, null).passThrough());
            for (int i = 1; i <= INTAKE_PASSES; i++) {
                if (i > 1) {
                    steps.add(new Drive("Back out " + (i - 1), pose(GARDEN_BACK_OUT), null, true, null, PUSH_TIMEOUT));
                }
                steps.add(new Drive("Push into corner " + i, pose(GARDEN_INTAKE), null, true, null, PUSH_TIMEOUT));
            }
            steps.add(new Wait("Intake corner (wait)", GARDEN_INTAKE_SECONDS, true));
        }
    }

    /*
     * Flower intake (same for every flower): drive to the approach, lower the flower servo and
     * wait for it to tip, drive in at normal speed, slow push FLOWER_SLOW_PUSH_IN, wait while
     * intaking, back out FLOWER_BACK_UP_IN (straight back along the heading), then lift.
     * approach / intake are red-side {x, y, heading deg}; the intake heading faces the flower.
     */
    private void addFlowerIntake(String label, double[] approach, double[] intake, double backUpIn) {
        double h = Math.toRadians(intake[2]);
        double back = FLOWER_SLOW_PUSH_IN - backUpIn; // net move along the heading
        double[] backPoint = {intake[0] + back * Math.cos(h), intake[1] + back * Math.sin(h), intake[2]};

        steps.add(new Drive("Drive to " + label, pose(approach), null, false, null));
        steps.add(new FlowerServo("Lower flower intake", BasicTeleopLM1.FLOWER_DOWN_POSITION, FLOWER_LOWER_SECONDS));
        steps.add(new Drive("Intake " + label + " (forward)", pose(intake), null, true, null));
        steps.add(new SlowPush("Slow push into " + label, FLOWER_SLOW_PUSH_IN, FLOWER_SLOW_POWER, FLOWER_SLOW_TIMEOUT));
        steps.add(new Wait("Intake " + label + " (wait)", FLOWER_INTAKE_WAIT_SECONDS, true));
        steps.add(new Drive("Back up " + backUpIn + " in", pose(backPoint), null, true, null));
        steps.add(new FlowerServo("Raise flower intake", BasicTeleopLM1.FLOWER_UP_POSITION, FLOWER_SERVO_SECONDS));
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
        final double timeout;
        final ElapsedTime timer = new ElapsedTime();
        // Pass-through: don't brake to a stop at the end; hand straight over to the next path
        // so the robot keeps its speed (only where the next path keeps going the same way).
        boolean passThrough = false;

        Drive passThrough() {
            passThrough = true;
            return this;
        }

        Drive(String name, Pose end, Pose bend, boolean runIntake, Pose spinFor) {
            this(name, end, bend, runIntake, spinFor, DRIVE_TIMEOUT);
        }

        Drive(String name, Pose end, Pose bend, boolean runIntake, Pose spinFor, double timeout) {
            super(name);
            this.end = end;
            this.bend = bend;
            this.runIntake = runIntake;
            this.spinFor = spinFor;
            this.timeout = timeout;
        }

        @Override
        void begin() {
            Pose start = follower.pose();
            // Turn the short way to the end heading.
            double endHeading = start.heading() + wrap(end.heading() - start.heading());
            Path path = (bend == null) ? line(start, end) : curve(start, bend, end);
            path = path.linear(start.heading(), endHeading);
            if (passThrough) {
                path = path.with(Constants.foresightConfig.brakeAtEnd.at(false));
            }
            follower.follow(path);
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
            if (passThrough) {
                // Next path starts right away, at full speed.
                return follower.atParametricEnd() || timer.seconds() > timeout;
            }
            boolean arrived = follower.atParametricEnd() && follower.distanceToEndpoint() < AT_END_INCHES;
            if (arrived || timer.seconds() > timeout) {
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

        final double minSpinUp;

        Shoot(String name, Pose cell) {
            this(name, cell, MIN_SPIN_UP_SECONDS);
        }

        Shoot(String name, Pose cell, double minSpinUp) {
            super(name);
            this.minSpinUp = minSpinUp;
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
                boolean spunUp = readyTimer.seconds() > minSpinUp;
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

    /*
     * Slow straight push forward (intake first) at a fixed low power, intake on, until the robot
     * has moved `inches` or `timeout` runs out. Then it holds where it stopped.
     */
    private class SlowPush extends Step {
        final double inches, power, timeout;
        final ElapsedTime timer = new ElapsedTime();
        Pose startPose;

        SlowPush(String name, double inches, double power, double timeout) {
            super(name);
            this.inches = inches;
            this.power = power;
            this.timeout = timeout;
        }

        @Override
        void begin() {
            startPose = follower.pose();
            follower.manual(power, 0, 0);
            timer.reset();
        }

        @Override
        boolean run() {
            intakePower = INTAKE_POWER;
            follower.manual(power, 0, 0);
            if (follower.pose().distance(startPose) >= inches || timer.seconds() > timeout) {
                follower.hold(follower.pose());
                return true;
            }
            return false;
        }
    }

    // Move the flower servo and give it time to get there (intake keeps running).
    private class FlowerServo extends Step {
        final double position;
        final double seconds;
        final ElapsedTime timer = new ElapsedTime();

        FlowerServo(String name, double position, double seconds) {
            super(name);
            this.position = position;
            this.seconds = seconds;
        }

        @Override
        void begin() {
            flowerServo.setPosition(position);
            timer.reset();
        }

        @Override
        boolean run() {
            intakePower = INTAKE_POWER;
            return timer.seconds() > seconds;
        }
    }

    /*
     * After the preloads: stay aimed and watch the preload cell's tags. Leaves as soon as the
     * tag is gone (normal route), or after it has stayed visible for TAG_WATCH_SECONDS
     * (shoot-again route). Adds the chosen route to the step list.
     */
    private class CheckPreloadCellTags extends Step {
        final ElapsedTime watchTimer = new ElapsedTime();
        final ElapsedTime sinceSeen = new ElapsedTime();

        CheckPreloadCellTags(String name) {
            super(name);
        }

        @Override
        void begin() {
            watchTimer.reset();
            sinceSeen.reset();
        }

        @Override
        boolean run() {
            if (preloadCellTagVisible()) {
                sinceSeen.reset();
            }
            if (sinceSeen.seconds() > TAG_LOST_GRACE_SECONDS) {
                routeNote = String.format("tag gone at %.1f s: normal route", autoTimer.seconds());
                addNormalRoute();
                return true;
            }
            if (watchTimer.seconds() >= TAG_WATCH_SECONDS) {
                routeNote = String.format("tag still seen at %.1f s: shoot-again route", autoTimer.seconds());
                addShootAgainRoute();
                return true;
            }
            return false;
        }
    }

    // True if the Limelight currently sees a tag of the cell the preloads were shot into.
    private boolean preloadCellTagVisible() {
        int first = alliance() == FieldLM1.Alliance.RED ? RED_PRELOAD_CELL_FIRST_TAG : BLUE_PRELOAD_CELL_FIRST_TAG;
        int last = alliance() == FieldLM1.Alliance.RED ? RED_PRELOAD_CELL_LAST_TAG : BLUE_PRELOAD_CELL_LAST_TAG;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > TAG_MAX_STALENESS_MS) {
            return false;
        }
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int id = tag.getFiducialId();
            if (id >= first && id <= last) {
                return true;
            }
        }
        return false;
    }

    // Runs once and finishes right away: used to pick the next part of the route at runtime.
    private class Decide extends Step {
        final Runnable action;

        Decide(String name, Runnable action) {
            super(name);
            this.action = action;
        }

        @Override
        boolean run() {
            action.run();
            return true;
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
