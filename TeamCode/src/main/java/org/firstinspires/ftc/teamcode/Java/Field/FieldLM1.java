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

    // Half the robot's front-to-back length and side-to-side width (18 in robot).
    public static final double ROBOT_HALF_LENGTH = 9;
    public static final double ROBOT_HALF_WIDTH = 9;

    /*
     * Hive cells (aim points). Each alliance's hive has two cells, one on the audience side of
     * center (y < 72) and one on the far side (y > 72), about 18.8 in apart.
     */
    public static final double CELL_HALF_SPACING = 9.4;
    public static final double RED_CELL_X = 60;
    public static final double BLUE_CELL_X = FIELD_SIZE - RED_CELL_X; // 84

    public static Pose audienceCell(Alliance a) {
        return new Pose(cellX(a), CENTER - CELL_HALF_SPACING);
    }

    public static Pose farCell(Alliance a) {
        return new Pose(cellX(a), CENTER + CELL_HALF_SPACING);
    }

    private static double cellX(Alliance a) {
        return a == Alliance.RED ? RED_CELL_X : BLUE_CELL_X;
    }

    /*
     * Auto start: front of the robot against your alliance wall, back facing the hive, on the
     * tile level with your hive cell that is farther from your loading zone.
     *   Red:  (9, 60) facing the red wall (heading 180 deg)
     *   Blue: (135, 84) facing the blue wall (heading 0 deg)
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
     *   Far corner:      the corner on the far side (top) of your alliance wall.
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
