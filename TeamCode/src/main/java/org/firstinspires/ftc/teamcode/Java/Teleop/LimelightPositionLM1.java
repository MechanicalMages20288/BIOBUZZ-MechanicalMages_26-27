package org.firstinspires.ftc.teamcode.Java.Teleop;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/*
 * LimelightPositionLM1: shows where the robot is on the field according to the Limelight's
 * AprilTag view (no Pinpoint). Gamepad 1 drives (same controls as the other teleops) so you
 * can move around and watch the position update. Nothing else runs.
 *
 * LIMELIGHT SETUP USED: configured as "limelight", AprilTag pipeline 0, BIOBUZZ field map
 * loaded on the Limelight. The Limelight does the position math itself (its "botpose").
 *
 * PIPELINE: at INIT this uploads the team's "apriltaglm1" pipeline (PIPELINE_JSON below) to
 * slot 0, including the camera mount (10 cm behind center, 8.5 cm to the side, 38 cm up,
 * tilted 45 deg, facing backward) and the 82.55 mm tag size.
 *
 * FIELD COORDINATES: the Limelight reports meters with (0, 0) at the field CENTER. This
 * converts to our Pedro frame (inches, red wall x = 0, audience side y = 0). If the field
 * map's axes are rotated from ours, set LL_FRAME_ROTATION_DEG (0, 90, 180 or 270): park in
 * the red audience corner (should read about 12, 12) and pick the value that matches.
 *
 *
 */
@TeleOp(name = "LimelightPositionLM1", group = "LM1")
public class LimelightPositionLM1 extends OpMode {

    public static String LIMELIGHT_NAME = "limelight";
    public static int APRILTAG_PIPELINE = 0;

    // Rotation from the Limelight field map's axes to ours (see note above).
    public static double LL_FRAME_ROTATION_DEG = 0;

    /*
     * Upload PIPELINE_JSON to the Limelight's pipeline slot at INIT, so the camera always runs
     * exactly these settings. Set to false to use whatever is saved on the Limelight instead.
     */
    public static boolean UPLOAD_PIPELINE_AT_INIT = true;

    /*
     * The team's "apriltaglm1" pipeline (.vpr export), with the camera height fixed to 0.38 m
     * (the export had rsu = 10.0, i.e. 10 meters). Key settings:
     *   fiducial_size 82.55 mm (3.25 in BIOBUZZ tags), 36h11 tags
     *   camera mount in robot space: rsf -0.10 m (behind center), rss -0.085 m (side),
     *   rsu 0.38 m (height), rspitch 45 deg, rsyaw 180 deg (facing backward), rsroll 0
     * To change a setting, edit it here (or re-export the .vpr from the Limelight and paste it).
     */
    public static final String PIPELINE_JSON = "{"
            + "\"area_max\":100.0,\"area_min\":0.0,\"area_similarity\":90.0,\"aspect_max\":20.0,\"aspect_min\":0.0,"
            + "\"barcode_type\":\"qrzx\",\"black_level\":0,\"blue_balance\":2482.0,\"botfloorsnap\":1,"
            + "\"botlength\":0.457,\"bottype\":\"swerve\",\"botwidth\":0.462,\"calibration_type\":0,"
            + "\"classifier_conf\":0.1,\"classifier_runtime\":\"coral\",\"contour_grouping\":0,\"contour_sort_final\":0,"
            + "\"convexity_max\":100.0,\"convexity_min\":0.0,\"corner_approx\":2.0,\"crop_focus\":0.0,"
            + "\"crop_x_max\":1.0,\"crop_x_min\":-1.0,\"crop_y_max\":1.0,\"crop_y_min\":-1.0,"
            + "\"cross_a_a\":0.0,\"cross_a_x\":0.0,\"cross_a_y\":0.0,\"cross_b_a\":1.0,\"cross_b_x\":0.0,\"cross_b_y\":0.0,"
            + "\"debugpipe\":0,\"desc\":\"apriltaglm1\",\"desired_contour_region\":0,\"detector_conf\":0.8,"
            + "\"detector_idfilters\":\"\",\"detector_runtime\":\"coral\",\"dilation_steps\":0,\"direction_filter\":0,"
            + "\"dual_close_sort_origin\":0,\"erosion_steps\":1,\"exposure\":939.0,\"fiducial_backend\":\"umich\","
            + "\"fiducial_denoise\":0.0,\"fiducial_idfilters\":\"\",\"fiducial_locfilters\":\"\","
            + "\"fiducial_qualitythreshold\":1.0,\"fiducial_resdiv\":2,\"fiducial_size\":82.55,\"fiducial_skip3d\":0,"
            + "\"fiducial_type\":\"aprilClassic36h11\",\"fiducial_vis_mode\":\"3dbotposefieldspace\",\"flicker\":0,"
            + "\"force_convex\":1,\"hue_max\":179,\"hue_min\":170,\"image_flip\":0,\"image_source\":0,\"img_to_show\":0,"
            + "\"intersection_filter\":0,\"invert_hue\":1,\"lcgain\":26.6,\"multigroup_max\":7,\"multigroup_min\":1,"
            + "\"multigroup_rejector\":0,\"nnp_rotate\":0,\"pipeline_led_enabled\":1,\"pipeline_led_power\":100,"
            + "\"pipeline_res\":3,\"pipeline_type\":\"pipe_fiducial\",\"python_snapscript_name\":\"\","
            + "\"quality_focus\":0.3,\"red_balance\":1452.0,\"reverse_morpho\":0,\"roi_x\":0.0,\"roi_y\":0.0,"
            + "\"rsf\":-0.1,\"rspitch\":45.0,\"rsroll\":0.0,\"rss\":-0.085,\"rsu\":0.38,\"rsyaw\":180.0,"
            + "\"sat_max\":255,\"sat_min\":240,\"send_corners\":1,\"send_json\":1,\"tsf\":0.0,\"tss\":0.0,\"tsu\":0.0,"
            + "\"val_max\":172,\"val_min\":142,\"x_outlier_miqr\":1.5,\"y_outlier_miqr\":1.5,\"yaw_latency_adjustment\":0.0"
            + "}";

    private boolean pipelineUploaded = false;

    private static final double METERS_TO_INCHES = 39.3701;
    private static final double FIELD_CENTER_IN = 72;

    private Limelight3A limelight;
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        if (UPLOAD_PIPELINE_AT_INIT) {
            pipelineUploaded = limelight.uploadPipeline(PIPELINE_JSON, APRILTAG_PIPELINE);
        }
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.setPollRateHz(100);
        limelight.start();

        // Drive motors: same names, directions and strafe fix as BasicTeleopLM1.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        telemetry.addLine("Limelight started. Press START, then drive around with gamepad 1.");
    }

    @Override
    public void init_loop() {
        showLimelight();
    }

    @Override
    public void loop() {
        drive(deadband(-gamepad1.left_stick_y), deadband(gamepad1.left_stick_x), deadband(gamepad1.right_stick_x));
        showLimelight();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    private void showLimelight() {
        telemetry.addData("Limelight", "%s, last update %d ms ago",
                limelight.isConnected() ? "connected" : "NOT CONNECTED", limelight.getTimeSinceLastUpdate());
        telemetry.addData("Pipeline", UPLOAD_PIPELINE_AT_INIT
                ? (pipelineUploaded ? "apriltaglm1 uploaded to slot " + APRILTAG_PIPELINE : "UPLOAD FAILED (using saved pipeline)")
                : "using saved pipeline " + APRILTAG_PIPELINE);

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            telemetry.addLine();
            telemetry.addData("Field position", "NO POSITION (no AprilTags in view)");
            telemetry.update();
            return;
        }

        // Position the Limelight worked out from the tags + its field map.
        Pose3D botpose = result.getBotpose();
        if (botpose != null && result.getBotposeTagCount() > 0) {
            Position m = botpose.getPosition().toUnit(DistanceUnit.METER);
            double yawDeg = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

            double r = Math.toRadians(LL_FRAME_ROTATION_DEG);
            double xIn = FIELD_CENTER_IN + (Math.cos(r) * m.x - Math.sin(r) * m.y) * METERS_TO_INCHES;
            double yIn = FIELD_CENTER_IN + (Math.sin(r) * m.x + Math.cos(r) * m.y) * METERS_TO_INCHES;
            double headingDeg = wrapDegrees(yawDeg + LL_FRAME_ROTATION_DEG);

            telemetry.addLine();
            telemetry.addData("Field position", "x %.1f in, y %.1f in, heading %.0f deg", xIn, yIn, headingDeg);
            telemetry.addData("Based on", "%d tag(s), avg distance %.0f in",
                    result.getBotposeTagCount(), result.getBotposeAvgDist() * METERS_TO_INCHES);
            telemetry.addData("Raw Limelight", "x %.2f m, y %.2f m, yaw %.0f deg (from field center)",
                    m.x, m.y, yawDeg);
        } else {
            telemetry.addLine();
            telemetry.addData("Field position", "tags seen but no position (check the field map)");
        }

        // Every tag in view, with its distance from the camera.
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        telemetry.addLine();
        telemetry.addData("Tags in view", tags.size());
        for (LLResultTypes.FiducialResult tag : tags) {
            Position t = tag.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.INCH);
            double distance = Math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z);
            telemetry.addData("  Tag " + tag.getFiducialId(), "%.0f in from camera  (%s)", distance, cellName(tag.getFiducialId()));
        }
        telemetry.addData("Latency", "%.0f ms", result.getCaptureLatency() + result.getTargetingLatency());
        telemetry.update();
    }

    // Which hive cell a tag ID belongs to (game manual 9.9).
    private static String cellName(int id) {
        if (id >= 30 && id <= 33) return "red far-side cell";
        if (id >= 34 && id <= 37) return "red audience-side cell";
        if (id >= 38 && id <= 41) return "blue audience-side cell";
        if (id >= 42 && id <= 45) return "blue far-side cell";
        return "not a BIOBUZZ tag";
    }

    // Same mecanum math as BasicTeleopLM1 (left-side strafe signs flipped for this robot).
    private void drive(double forward, double strafe, double rotate) {
        double lf = forward - strafe + rotate;
        double rf = forward - strafe - rotate;
        double lb = forward + strafe + rotate;
        double rb = forward + strafe - rotate;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb))));
        leftFrontDrive.setPower(lf / max);
        rightFrontDrive.setPower(rf / max);
        leftBackDrive.setPower(lb / max);
        rightBackDrive.setPower(rb / max);
    }

    private static double deadband(double v) {
        return Math.abs(v) < 0.05 ? 0 : v;
    }

    private static double wrapDegrees(double d) {
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        return d;
    }
}
