package org.firstinspires.ftc.teamcode.Autos;

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Java.Field.FieldLM1;
import org.firstinspires.ftc.teamcode.Java.Field.PoseStorage;
import org.firstinspires.ftc.teamcode.Java.Shooter.FireFeedforward;
import org.firstinspires.ftc.teamcode.Java.Teleop.BasicTeleopLM1;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.Java.Shooter.LauncherVoltageComp;

/*
 * Preload-only auto: shoot the 4 preloads from the start spot, then park in the loading zone.
 * RedPreloadParkAutoLM1 / BluePreloadParkAutoLM1 only pick the alliance.
 *
 * Written for RED; blue is the same thing rotated 180 degrees around the field center.
 *
 * SETUP: same start as RedAutoLM1 / BlueAutoLM1 (PedroAutoLM1.START): red (58, 12), FRONT
 * against the audience wall, BACK (shooter) toward the hive. Blue: rotated (86, 132), front
 * against the far wall.
 *
 * SEQUENCE
 *   1. SHOOT  turn in place so the back points at our near cell (red audience-side cell /
 *             blue far-side cell), spin up to PedroAutoLM1.SHOT_VELOCITY, and once aimed and
 *             steadily at speed, feed all 4 balls.
 *   2. PARK   slide off the wall, drive up along our alliance wall (shooter first, same red-side
 *             points as PedroAutoLM1), then into the loading zone.
 *
 * The start spot, shot speed and fire feedforward come from PedroAutoLM1, so tuning those there
 * tunes this auto too.
 */
public abstract class PreloadParkAutoLM1 extends OpMode {

    protected abstract FieldLM1.Alliance alliance();

    // ---------- Timing ----------
    public static double MIN_SPIN_UP_SECONDS = 2.0;  // flywheel always gets at least this long
    // The flywheel must stay at speed this long before feeding (fully spun up, not passing through).
    public static double AT_SPEED_HOLD_SECONDS = 0.3;
    public static double SHOOT_READY_TIMEOUT = 5.0;  // safety net: feed anyway after this long
    public static double FEED_SECONDS = 3;           // intake + transfer time for all 4 balls
    public static double DRIVE_TIMEOUT = 4.0;        // give up on a park leg after this long
    public static double AT_END_INCHES = 2.0;

    // ---------- Park route, RED side (x, y in inches). Heading stays -90: shooter first. ----------
    // 1. Slide off the audience wall toward the red wall, clear of the hive.
    // 2-3. Up the red side (same points as PedroAutoLM1 paths 3 and 4).
    // 4. Into the loading zone (same spot as PedroAutoLM1.PARK).
    public static double[][] PARK_ROUTE = {
            {30, 20},
            {26.1485, 63.593},
            {27.3506, 117.0332},
            {10, 115},
    };
    public static double PARK_HEADING_DEG = -90;

    private enum State { SHOOT, PARK, DONE }

    private Follower follower;
    private DcMotorEx launcher;
    private LauncherVoltageComp launcherVoltage;
    private DcMotor intake;
    private DcMotor transfer;
    private Servo flowerServo;

    private final FireFeedforward fireFeedforward = new FireFeedforward(PedroAutoLM1.AUTO_FIRE_FEEDFORWARD);
    private final ElapsedTime autoTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime atSpeedTimer = new ElapsedTime();
    private boolean wasAtSpeed = false;

    private State state = State.SHOOT;
    private Pose startPose;
    private Pose cell;
    private Pose aimed;
    private boolean feeding = false;
    private String shotNote = "";
    private Pose[] parkLegs;
    private int parkLeg = 0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        // Recalibrate the Pinpoint's gyro. KEEP THE ROBOT STILL during INIT.
        hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get()).recalibrateIMU();

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

        double[] s = PedroAutoLM1.START;
        startPose = toAlliance(new Pose(s[0], s[1], Math.toRadians(s[2])));
        // Red: audience-side cell. Blue: far-side cell (red's near side rotated 180 deg).
        cell = alliance() == FieldLM1.Alliance.RED
                ? FieldLM1.audienceCell(FieldLM1.Alliance.RED)
                : FieldLM1.farCell(FieldLM1.Alliance.BLUE);
        aimed = aimPose(startPose, cell);

        parkLegs = new Pose[PARK_ROUTE.length];
        for (int i = 0; i < PARK_ROUTE.length; i++) {
            parkLegs[i] = toAlliance(new Pose(PARK_ROUTE[i][0], PARK_ROUTE[i][1], Math.toRadians(PARK_HEADING_DEG)));
        }
        follower.setPose(startPose);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Alliance", alliance());
        telemetry.addData("Place robot at", "x %.0f, y %.0f, front against the %s wall, back toward the hive",
                startPose.x(), startPose.y(), alliance() == FieldLM1.Alliance.RED ? "AUDIENCE" : "FAR");
        telemetry.addData("Shot", "%.0f ticks/s at the %s, %.0f in away",
                PedroAutoLM1.SHOT_VELOCITY,
                alliance() == FieldLM1.Alliance.RED ? "red audience-side cell" : "blue far-side cell",
                startPose.distance(cell));
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(startPose);
        flowerServo.setPosition(BasicTeleopLM1.FLOWER_UP_POSITION);
        // Turn in place so the back points at the cell, and hold there while spinning up.
        follower.hold(aimed);
        autoTimer.reset();
        enter(State.SHOOT);
    }

    @Override
    public void loop() {
        follower.update();

        double intakePower = 0;
        // Not shooting: transfer turns slowly backward so balls can't slip into the flywheel.
        double transferPower = PedroAutoLM1.TRANSFER_HOLD_POWER;
        boolean firing = false;

        switch (state) {
            case SHOOT:
                if (!feeding) {
                    boolean isAimed = Math.abs(Math.toDegrees(wrap(aimed.heading() - follower.pose().heading())))
                            <= PedroAutoLM1.AIM_TOLERANCE_DEG;
                    boolean atSpeed = Math.abs(launcher.getVelocity() - PedroAutoLM1.SHOT_VELOCITY)
                            <= PedroAutoLM1.LAUNCHER_TOLERANCE;
                    if (atSpeed && !wasAtSpeed) {
                        atSpeedTimer.reset();
                    }
                    wasAtSpeed = atSpeed;
                    boolean steadyAtSpeed = atSpeed && atSpeedTimer.seconds() >= AT_SPEED_HOLD_SECONDS;
                    boolean spunUp = autoTimer.seconds() >= MIN_SPIN_UP_SECONDS;
                    boolean gaveUp = stateTimer.seconds() > SHOOT_READY_TIMEOUT;
                    if ((isAimed && steadyAtSpeed && spunUp) || gaveUp) {
                        if (gaveUp) {
                            shotNote = "not aimed / at speed in time, fed anyway";
                        }
                        feeding = true;
                        stateTimer.reset();
                    }
                } else {
                    intakePower = PedroAutoLM1.INTAKE_FIRE_POWER;
                    transferPower = PedroAutoLM1.TRANSFER_FEED_POWER;
                    firing = true;
                    if (stateTimer.seconds() > FEED_SECONDS) {
                        parkLeg = 0;
                        enter(State.PARK);
                        startParkLeg();
                    }
                }
                break;

            case PARK:
                boolean lastLeg = parkLeg == parkLegs.length - 1;
                boolean arrived = lastLeg
                        ? follower.atParametricEnd() && follower.distanceToEndpoint() < AT_END_INCHES
                        : follower.atParametricEnd();
                if (arrived || stateTimer.seconds() > DRIVE_TIMEOUT) {
                    if (lastLeg) {
                        follower.hold(parkLegs[parkLeg]);
                        enter(State.DONE);
                    } else {
                        parkLeg++;
                        stateTimer.reset();
                        startParkLeg();
                    }
                }
                break;

            case DONE:
                break;
        }

        double flywheel = state == State.SHOOT ? PedroAutoLM1.SHOT_VELOCITY : 0;
        launcherVoltage.update();
        launcher.setVelocity(fireFeedforward.command((int) Math.round(flywheel), launcher.getVelocity(), firing));
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        Pose p = follower.pose();
        PoseStorage.save(p, alliance());

        telemetry.addData("State", state == State.PARK
                ? String.format("PARK (leg %d/%d)", parkLeg + 1, parkLegs.length) : state.toString());
        telemetry.addData("Time", "%.1f s", autoTimer.seconds());
        if (state == State.SHOOT) {
            telemetry.addData("Shooting", feeding ? "FEEDING" : "aiming + waiting for full speed");
        }
        if (!shotNote.isEmpty()) {
            telemetry.addData("Shot note", shotNote);
        }
        telemetry.addData("Flywheel", "target %.0f, actual %.0f ticks/s", flywheel, launcher.getVelocity());
        telemetry.addData("Pose", "x %.1f, y %.1f, heading %.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
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

    private void enter(State next) {
        state = next;
        stateTimer.reset();
        feeding = false;
        wasAtSpeed = false;
    }

    // Drive the next park leg in a straight line, turning the short way to the park heading.
    // Every leg but the last keeps its speed into the next one instead of braking.
    private void startParkLeg() {
        Pose from = follower.pose();
        Pose to = parkLegs[parkLeg];
        double endHeading = from.heading() + wrap(to.heading() - from.heading());
        Path path = line(from, to).linear(from.heading(), endHeading);
        if (parkLeg < parkLegs.length - 1) {
            path = path.with(Constants.foresightConfig.brakeAtEnd.at(false));
        }
        follower.follow(path);
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

    private static double wrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
