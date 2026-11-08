package org.firstinspires.ftc.teamcode.Java.Field;

import com.pedropathing.math.Pose;

/*
 * BIOBUZZ field positions used by the LM1 autos and teleops, in Pedro's field coordinates.
 *
 * FIELD FRAME (looking down at the field from above, audience at the bottom):
 *   x: 0 at the RED alliance wall (left)   -> 144 at the BLUE alliance wall (right)
 *   y: 0 at the AUDIENCE side wall (bottom) -> 144 at the far side wall (top)
 *   heading: 0 = pointing at the blue wall (+x), counterclockwise is positive, in radians.
 * All distances are inches. The field is the same for both alliances rotated 180 degrees.
 *
 * Everything here is an ESTIMATE from the game manual / field drawing. Measure on a real field
 * and adjust: the cell positions matter most for aiming.
 */
public final class FieldLM1 {

    public enum Alliance { RED, BLUE }

    public static final double FIELD_SIZE = 144;
    public static final double CENTER = FIELD_SIZE / 2;

    /*
     * Robot center to its edges, measured by the team in the Pedro visualizer: with the front
     * against a wall and the side against the other wall, the robot center is at (12, 12).
     * (The shooter edge is ~10 in behind the center, the other way.)
     */
    public static final double ROBOT_HALF_LENGTH = 12; // center to front edge
    public static final double ROBOT_HALF_WIDTH = 12;  // center to side edge

    /*
     * Hive cells (aim points), measured in the Pedro visualizer. Each alliance's hive has an
     * audience-side cell (y < 72) and a far-side cell (y > 72).
     */
    public static final Pose RED_AUDIENCE_CELL = new Pose(57.88, 59.26);
    public static final Pose RED_FAR_CELL = new Pose(57.57, 88.02);
    public static final Pose BLUE_AUDIENCE_CELL = new Pose(84.15, 55.23);
    public static final Pose BLUE_FAR_CELL = new Pose(84.78, 81.4);

    public static Pose audienceCell(Alliance a) {
        return a == Alliance.RED ? RED_AUDIENCE_CELL : BLUE_AUDIENCE_CELL;
    }

    public static Pose farCell(Alliance a) {
        return a == Alliance.RED ? RED_FAR_CELL : BLUE_FAR_CELL;
    }

    /*
     * Auto start: front of the robot against your alliance wall, back facing the hive, on the
     * tile level with your hive cell that is farther from your loading zone.
     *   Red:  (12, 60) facing the red wall (heading 180 deg)
     *   Blue: (132, 84) facing the blue wall (heading 0 deg)
     */
    public static Pose autoStartPose(Alliance a) {
        if (a == Alliance.RED) {
            return new Pose(ROBOT_HALF_LENGTH, 60, Math.PI);
        }
        return new Pose(FIELD_SIZE - ROBOT_HALF_LENGTH, 84, 0);
    }

    /*
     * Relocalization corners: push the robot into the corner with its FRONT flat against your
     * alliance wall and its side flat against the side wall, then press the button.
     *   Audience corner: the corner on the audience side (bottom) of your alliance wall.
     *                    Red (12, 12, 180 deg), Blue (132, 12, 0 deg).
     *   Far corner:      the corner on the far side (top) of your alliance wall.
     *                    Red (12, 132, 180 deg), Blue (132, 132, 0 deg).
     */
    public static Pose audienceCornerPose(Alliance a) {
        if (a == Alliance.RED) {
            return new Pose(ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH, Math.PI);
        }
        return new Pose(FIELD_SIZE - ROBOT_HALF_LENGTH, ROBOT_HALF_WIDTH, 0);
    }

    public static Pose farCornerPose(Alliance a) {
        if (a == Alliance.RED) {
            return new Pose(ROBOT_HALF_LENGTH, FIELD_SIZE - ROBOT_HALF_WIDTH, Math.PI);
        }
        return new Pose(FIELD_SIZE - ROBOT_HALF_LENGTH, FIELD_SIZE - ROBOT_HALF_WIDTH, 0);
    }

    private FieldLM1() {
    }
}
