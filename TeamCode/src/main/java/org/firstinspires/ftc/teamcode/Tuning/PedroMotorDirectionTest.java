package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/*
 * Checks the drive motors EXACTLY as Pedro sees them: the names and directions come straight
 * from Constants.drivetrainConfig, and the combined moves use Pedro's (standard) mecanum math.
 * PUT THE ROBOT ON A BOX so the wheels are off the floor.
 *
 * GAMEPAD 1, hold a button:
 *   ONE WHEEL AT A TIME (each should spin so it would push the robot FORWARD,
 *   forward = the INTAKE side, and it should be the wheel at that corner):
 *     Y  Pedro "front left"      B  Pedro "front right"
 *     X  Pedro "back left"       A  Pedro "back right"
 *
 *   ALL FOUR WHEELS (Pedro's mecanum math), robot on the floor or on a box:
 *     D-pad up     drive forward   (all 4 wheels spin forward)
 *     D-pad left   strafe left     (front-left + back-right spin BACKWARD,
 *                                   front-right + back-left spin FORWARD)
 *     D-pad right  turn left       (both left wheels backward, both right wheels forward)
 *
 * Left bumper (hold) = half power, for a slower look.
 */
@TeleOp(name = "Pedro Motor Direction Test", group = "Tuning")
public class PedroMotorDirectionTest extends OpMode {

    public static double POWER = 0.3;

    private DcMotor fl, fr, bl, br;
    private String flName, frName, blName, brName;
    private DcMotorSimple.Direction flDir, frDir, blDir, brDir;

    @Override
    public void init() {
        flName = Constants.drivetrainConfig.frontLeftName.get();
        frName = Constants.drivetrainConfig.frontRightName.get();
        blName = Constants.drivetrainConfig.backLeftName.get();
        brName = Constants.drivetrainConfig.backRightName.get();
        flDir = Constants.drivetrainConfig.frontLeftDirection.get();
        frDir = Constants.drivetrainConfig.frontRightDirection.get();
        blDir = Constants.drivetrainConfig.backLeftDirection.get();
        brDir = Constants.drivetrainConfig.backRightDirection.get();

        fl = setUp(flName, flDir);
        fr = setUp(frName, frDir);
        bl = setUp(blName, blDir);
        br = setUp(brName, brDir);
        telemetry.addLine("Robot on a box. Press START, then hold Y / B / X / A or the d-pad.");
    }

    private DcMotor setUp(String name, DcMotorSimple.Direction dir) {
        DcMotor m = hardwareMap.get(DcMotor.class, name);
        m.setDirection(dir);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return m;
    }

    @Override
    public void loop() {
        double p = gamepad1.left_bumper ? POWER / 2 : POWER;
        double flP = 0, frP = 0, blP = 0, brP = 0;
        String test = "none (hold a button)";

        if (gamepad1.y) { flP = p; test = "FRONT LEFT only -> should spin forward, at the front-left corner"; }
        else if (gamepad1.b) { frP = p; test = "FRONT RIGHT only -> should spin forward, at the front-right corner"; }
        else if (gamepad1.x) { blP = p; test = "BACK LEFT only -> should spin forward, at the back-left corner"; }
        else if (gamepad1.a) { brP = p; test = "BACK RIGHT only -> should spin forward, at the back-right corner"; }
        else if (gamepad1.dpad_up) { flP = frP = blP = brP = p; test = "DRIVE FORWARD (Pedro math)"; }
        else if (gamepad1.dpad_left) {
            // Pedro / standard mecanum, strafe left: FL = -s, FR = +s, BL = +s, BR = -s
            flP = -p; frP = p; blP = p; brP = -p;
            test = "STRAFE LEFT (Pedro math)";
        } else if (gamepad1.dpad_right) {
            // Turn left (counterclockwise): left wheels back, right wheels forward
            flP = -p; blP = -p; frP = p; brP = p;
            test = "TURN LEFT (Pedro math)";
        }

        fl.setPower(flP);
        fr.setPower(frP);
        bl.setPower(blP);
        br.setPower(brP);

        telemetry.addData("Testing", test);
        telemetry.addLine();
        telemetry.addLine("Pedro config (from Constants.java):");
        telemetry.addData("  Front left ", "%s, %s, power %.2f", flName, flDir, flP);
        telemetry.addData("  Front right", "%s, %s, power %.2f", frName, frDir, frP);
        telemetry.addData("  Back left  ", "%s, %s, power %.2f", blName, blDir, blP);
        telemetry.addData("  Back right ", "%s, %s, power %.2f", brName, brDir, brP);
        telemetry.addLine();
        telemetry.addLine("Y=FL  B=FR  X=BL  A=BR | dpad up fwd, left strafe L, right turn L");
    }

    @Override
    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
}
