package org.firstinspires.ftc.teamcode.Autos;

import static com.pedropathing.api.Paths.line;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
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

/*
 * Far-wall "wait for the tip" auto. RedFarWallAutoLM1 / BlueFarWallAutoLM1 only pick the alliance.
 *
 * IDEA: our far-side hive cell starts tipped away from us. When the other alliance tips it, its
 * AprilTags turn toward our camera. We sit against the far wall, already lined up, with the
 * flywheel spinning, and fire the 4 preloads the moment the Limelight sees that cell's tags.
 *
 * "FAR" IS PER ALLIANCE: the field is the same for both alliances rotated 180 degrees, so
 * each alliance's far wall and cell are the ones away from its own side:
 *   Red:  starts against the far wall (y = 144), shoots at the red far-side cell (tags 30-33).
 *   Blue: starts against the audience wall (y = 0), shoots at the blue audience-side cell
 *         (tags 38-41). Everything for blue is red's setup rotated 180 degrees.
 *
 * SETUP: FRONT of the robot flat against that wall, BACK (shooter + Limelight) facing the hive,
 * robot centered on the cell (INIT telemetry shows the exact x). The robot does not turn or
 * move to shoot.
 *
 * SEQUENCE
 *   1. WAIT    flywheel spins up; watch for that cell's tags (game manual 9.9).
 *   2. SHOOT   tags seen for TAG_CONFIRM_LOOPS loops in a row: wait SHOT_DELAY_AFTER_TAGS_SECONDS
 *              (lets the cell finish tipping), then wait until the flywheel has held its target
 *              speed for AT_SPEED_HOLD_SECONDS, then feed all 4 balls.
 *              (If no tags by SHOOT_ANYWAY_SECONDS: same, but without the delay.)
 *   3. PARK    drive to our loading zone.
 */
public abstract class FarWallTagAutoLM1 extends OpMode {

    protected abstract FieldLM1.Alliance alliance();

    // ---------- Tags ----------
    // Tags on the cell each alliance shoots at (game manual 9.9).
    public static int RED_CELL_FIRST_TAG = 30, RED_CELL_LAST_TAG = 33;   // red far-side cell
    public static int BLUE_CELL_FIRST_TAG = 38, BLUE_CELL_LAST_TAG = 41; // blue audience-side cell
    // Tags must be seen this many loops in a row before shooting (ignores one-frame glitches).
    public static int TAG_CONFIRM_LOOPS = 3;
    // Ignore Limelight frames older than this.
    public static double MAX_STALENESS_MS = 200;

    // ---------- Timing ----------
    // If the tags still haven't shown up by this time, shoot anyway (leaves time to park).
    public static double SHOOT_ANYWAY_SECONDS = 18;
    // After the tags are seen, wait this long before shooting (lets the cell finish tipping).
    public static double SHOT_DELAY_AFTER_TAGS_SECONDS = 3.0;
    public static double MIN_SPIN_UP_SECONDS = 2.0; // flywheel always gets at least this long
    // The flywheel must stay within LAUNCHER_TOLERANCE of its target this long before feeding,
    // so it's fully and steadily up to speed (not just passing through it).
    public static double AT_SPEED_HOLD_SECONDS = 0.3;
    // Safety net only: if the flywheel STILL isn't at speed this long after the delay (e.g. very
    // low battery), feed anyway so the auto doesn't sit there and miss the park.
    public static double SHOOT_READY_TIMEOUT = 6.0;
    public static double FEED_SECONDS = 3;          // intake + transfer time for all 4 balls
    public static double DRIVE_TIMEOUT = 5.0;       // give up on a park leg after this long
    public static double AT_END_INCHES = 2.0;

    // ---------- Shooter ----------
    // 0 = pick the speed from the distance to the cell (FlywheelSpeedModel). Otherwise a fixed speed.
    public static double SHOT_VELOCITY_OVERRIDE = 0;
    // Added to the distance-based speed (all autos shoot 30 ticks/s slower).
    public static double MODEL_SPEED_TRIM = -30;
    public static double LAUNCHER_TOLERANCE = 50;
    public static double INTAKE_FIRE_POWER = 1;
    public static double TRANSFER_FEED_POWER = 1;

    // ---------- Positions, written for RED (FieldLM1 frame, inches); blue = rotated 180 deg ----------
    // Robot center is 12 in from its front edge, so against the far wall y = 144 - 12.
    // Heading 90 = front toward the far wall. Start x is taken from the cell, so it's lined up.
    public static double START_Y = FieldLM1.FIELD_SIZE - FieldLM1.ROBOT_HALF_LENGTH;
    public static double START_HEADING_DEG = 90;
    // Loading zone: same park spot as PedroAutoLM1 (blue gets it rotated: 131, 29).
    // Short slide along the wall, no turning.
    public static double[] PARK = {13, 115};

    private enum State { WAIT_FOR_TAGS, SHOOT, PARK, DONE }

    private Follower follower;
    private Limelight3A limelight;
    private DcMotorEx launcher;
    private LauncherVoltageComp launcherVoltage;
    private DcMotor intake;
    private DcMotor transfer;
    private Servo flowerServo;

    private final FireFeedforward fireFeedforward = new FireFeedforward(PedroAutoLM1.AUTO_FIRE_FEEDFORWARD);
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime atSpeedTimer = new ElapsedTime(); // how long the flywheel has been at speed
    private boolean wasAtSpeed = false;
    private double shotDelay = 0; // delay before shooting for this shot (0 when shooting anyway)

    private State state = State.WAIT_FOR_TAGS;
    private Pose startPose;
    private Pose cell;
    private double shotVelocity;
    private int tagStreak = 0;
    private boolean tagsSeenNow = false;
    private String shootReason = "";
    private boolean feeding = false;
    private Pose[] parkLegs;
    private int parkLeg = 0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        // Recalibrate the Pinpoint's gyro. KEEP THE ROBOT STILL during INIT.
        hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get()).recalibrateIMU();

        limelight = hardwareMap.get(Limelight3A.class, LimelightPositionLM1.LIMELIGHT_NAME);
        if (LimelightPositionLM1.UPLOAD_PIPELINE_AT_INIT) {
            limelight.uploadPipeline(LimelightPositionLM1.PIPELINE_JSON, LimelightPositionLM1.APRILTAG_PIPELINE);
        }
        limelight.pipelineSwitch(LimelightPositionLM1.APRILTAG_PIPELINE);
        limelight.setPollRateHz(100);
        limelight.start();

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Launcher PIDF (40, 0, 0, 14.4) with battery voltage compensation on F.
        launcherVoltage = new LauncherVoltageComp(hardwareMap, launcher);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(BRAKE);
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setZeroPowerBehavior(BRAKE);
        flowerServo = hardwareMap.get(Servo.class, BasicTeleopLM1.FLOWER_SERVO_NAME);

        // Red: far-side cell. Blue: audience-side cell (red's far side rotated 180 deg).
        cell = alliance() == FieldLM1.Alliance.RED
                ? FieldLM1.farCell(FieldLM1.Alliance.RED)
                : FieldLM1.audienceCell(FieldLM1.Alliance.BLUE);
        // Start against that wall, lined up with the cell's x.
        Pose wallSpot = toAlliance(new Pose(0, START_Y, Math.toRadians(START_HEADING_DEG)));
        startPose = new Pose(cell.x(), wallSpot.y(), wallSpot.heading());
        shotVelocity = SHOT_VELOCITY_OVERRIDE > 0
                ? SHOT_VELOCITY_OVERRIDE
                : FlywheelSpeedModel.velocityForDistance(startPose.distance(cell)) + MODEL_SPEED_TRIM;

        parkLegs = new Pose[]{toAlliance(new Pose(PARK[0], PARK[1], Math.toRadians(START_HEADING_DEG)))};
        follower.setPose(startPose);
    }

    @Override
    public void init_loop() {
        // Show whether the tags are visible right now, to help with setup.
        checkTags();
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Place robot at", "x %.1f, front against the %s wall, back toward the hive",
                startPose.x(), alliance() == FieldLM1.Alliance.RED ? "FAR" : "AUDIENCE");
        telemetry.addData("Our cell tags", "%d-%d  %s", firstTag(), lastTag(), tagsSeenNow ? "VISIBLE NOW" : "not visible");
        telemetry.addData("Limelight", limelight.isConnected() ? "connected" : "NOT CONNECTED");
        telemetry.addData("Shot speed", "%.0f ticks/s (%.0f in to cell)", shotVelocity, startPose.distance(cell));
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(startPose);
        flowerServo.setPosition(BasicTeleopLM1.FLOWER_UP_POSITION);
        autoTimer.reset();
        stateTimer.reset();
        tagStreak = 0;
        state = State.WAIT_FOR_TAGS;
    }

    @Override
    public void loop() {
        follower.update();
        checkTags();

        double intakePower = 0;
        double transferPower = 0;
        boolean firing = false;

        switch (state) {
            case WAIT_FOR_TAGS:
                if (tagStreak >= TAG_CONFIRM_LOOPS) {
                    shootReason = String.format("tags seen at %.1f s", autoTimer.seconds());
                    shotDelay = SHOT_DELAY_AFTER_TAGS_SECONDS;
                    enter(State.SHOOT);
                } else if (autoTimer.seconds() > SHOOT_ANYWAY_SECONDS) {
                    shootReason = String.format("no tags by %.0f s, shooting anyway", SHOOT_ANYWAY_SECONDS);
                    shotDelay = 0;
                    enter(State.SHOOT);
                }
                break;

            case SHOOT:
                if (!feeding) {
                    // Track how long the flywheel has stayed at its target speed.
                    boolean atSpeed = Math.abs(launcher.getVelocity() - shotVelocity) <= LAUNCHER_TOLERANCE;
                    if (atSpeed && !wasAtSpeed) {
                        atSpeedTimer.reset();
                    }
                    wasAtSpeed = atSpeed;
                    boolean steadyAtSpeed = atSpeed && atSpeedTimer.seconds() >= AT_SPEED_HOLD_SECONDS;

                    boolean delayDone = stateTimer.seconds() >= shotDelay;
                    boolean spunUp = autoTimer.seconds() >= MIN_SPIN_UP_SECONDS;
                    boolean gaveUpWaiting = stateTimer.seconds() > shotDelay + SHOOT_READY_TIMEOUT;
                    if (delayDone && ((steadyAtSpeed && spunUp) || gaveUpWaiting)) {
                        if (gaveUpWaiting && !steadyAtSpeed) {
                            shootReason += " (flywheel never reached speed, fed anyway)";
                        }
                        feeding = true;
                        stateTimer.reset();
                    }
                } else {
                    intakePower = INTAKE_FIRE_POWER;
                    transferPower = TRANSFER_FEED_POWER;
                    firing = true;
                    if (stateTimer.seconds() > FEED_SECONDS) {
                        parkLeg = 0;
                        enter(State.PARK);
                        startParkLeg();
                    }
                }
                break;

            case PARK:
                boolean arrived = follower.atParametricEnd() && follower.distanceToEndpoint() < AT_END_INCHES;
                if (arrived || stateTimer.seconds() > DRIVE_TIMEOUT) {
                    parkLeg++;
                    if (parkLeg < parkLegs.length) {
                        stateTimer.reset();
                        startParkLeg();
                    } else {
                        follower.hold(parkLegs[parkLegs.length - 1]);
                        enter(State.DONE);
                    }
                }
                break;

            case DONE:
                break;
        }

        // Flywheel spins the whole auto until parking, so it's ready the moment the tags appear.
        double flywheel = (state == State.WAIT_FOR_TAGS || state == State.SHOOT) ? shotVelocity : 0;
        launcherVoltage.update();
        launcher.setVelocity(fireFeedforward.command((int) Math.round(flywheel), launcher.getVelocity(), firing));
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        Pose p = follower.pose();
        PoseStorage.save(p, alliance());

        telemetry.addData("State", state);
        telemetry.addData("Time", "%.1f s", autoTimer.seconds());
        telemetry.addData("Our cell tags", "%s (streak %d)", tagsSeenNow ? "VISIBLE" : "not visible", tagStreak);
        if (!shootReason.isEmpty()) {
            telemetry.addData("Shot", shootReason);
        }
        if (state == State.SHOOT) {
            double delayLeft = shotDelay - stateTimer.seconds();
            telemetry.addData("Shooting", feeding ? "FEEDING"
                    : delayLeft > 0 ? String.format("waiting %.1f s after tags", delayLeft)
                    : "waiting for flywheel to be fully at speed");
        }
        telemetry.addData("Flywheel", "target %.0f, actual %.0f ticks/s", flywheel, launcher.getVelocity());
        telemetry.addData("Pose", "x %.1f, y %.1f, heading %.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        PoseStorage.save(follower.pose(), alliance());
        follower.stop();
        limelight.stop();
        launcher.setVelocity(0);
        intake.setPower(0);
        transfer.setPower(0);
    }

    // Updates tagsSeenNow and the streak of loops in a row with one of our cell's tags in view.
    private void checkTags() {
        tagsSeenNow = false;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid() && result.getStaleness() <= MAX_STALENESS_MS) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                int id = tag.getFiducialId();
                if (id >= firstTag() && id <= lastTag()) {
                    tagsSeenNow = true;
                    break;
                }
            }
        }
        tagStreak = tagsSeenNow ? tagStreak + 1 : 0;
    }

    private void enter(State next) {
        state = next;
        stateTimer.reset();
        feeding = false;
        wasAtSpeed = false;
    }

    // Drive the next park leg in a straight line without turning.
    private void startParkLeg() {
        Pose from = follower.pose();
        Pose to = parkLegs[parkLeg];
        follower.follow(line(from, to).constant(from.heading()));
    }

    // Blue = red rotated 180 degrees around the field center.
    private Pose toAlliance(Pose red) {
        if (alliance() == FieldLM1.Alliance.RED) {
            return red;
        }
        double size = FieldLM1.FIELD_SIZE;
        double heading = red.heading() + Math.PI;
        while (heading > Math.PI) heading -= 2 * Math.PI;
        return new Pose(size - red.x(), size - red.y(), heading);
    }

    private int firstTag() {
        return alliance() == FieldLM1.Alliance.RED ? RED_CELL_FIRST_TAG : BLUE_CELL_FIRST_TAG;
    }

    private int lastTag() {
        return alliance() == FieldLM1.Alliance.RED ? RED_CELL_LAST_TAG : BLUE_CELL_LAST_TAG;
    }
}
