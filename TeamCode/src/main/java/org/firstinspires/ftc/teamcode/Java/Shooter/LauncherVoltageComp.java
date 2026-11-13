package org.firstinspires.ftc.teamcode.Java.Shooter;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Battery voltage compensation for the launcher.
 *
 * The launcher's velocity PIDF uses F (feedforward) to supply most of its power. F was tuned at
 * a fresh battery; as the battery drops, the same F gives less real power, so the flywheel runs
 * slower and recovers slower between balls. This scales F by NOMINAL_VOLTAGE / battery voltage
 * so the wheel gets the same real power at any charge:
 *     F used = BASE_F x NOMINAL_VOLTAGE / battery
 * e.g. BASE_F 14.4 at 13.0 V nominal: at 12.0 V F becomes 15.6.
 *
 * Every teleop and auto creates one of these instead of setting the PIDF directly, and calls
 * update() each loop. The voltage is smoothed and the PIDF is only re-sent when F changes.
 */
public class LauncherVoltageComp {

    public static boolean ENABLED = true;

    // Launcher PIDF as tuned (same as before).
    public static double P = 40, I = 0, D = 0, BASE_F = 14.4;

    // Battery voltage BASE_F was tuned at. At this voltage F is unchanged.
    public static double NOMINAL_VOLTAGE = 13.0;

    // Keep the scaled F within these limits (protects against a bad voltage reading).
    public static double MIN_SCALE = 0.85, MAX_SCALE = 1.30;

    public static double UPDATE_SECONDS = 0.25;   // how often to re-check the battery
    public static double VOLTAGE_FILTER = 0.3;    // smoothing (0-1, higher = follows faster)
    public static double MIN_F_CHANGE = 0.05;     // only re-send the PIDF when F moves this much

    private final DcMotorEx launcher;
    private final HardwareMap hardwareMap;
    private final ElapsedTime timer = new ElapsedTime();
    private double voltage = Double.NaN;
    private double appliedF = Double.NaN;

    public LauncherVoltageComp(HardwareMap hardwareMap, DcMotorEx launcher) {
        this.hardwareMap = hardwareMap;
        this.launcher = launcher;
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        voltage = readVoltage();
        apply();
    }

    // Call once per loop.
    public void update() {
        if (timer.seconds() < UPDATE_SECONDS) return;
        timer.reset();
        double v = readVoltage();
        if (!Double.isNaN(v)) {
            voltage = Double.isNaN(voltage) ? v : voltage + VOLTAGE_FILTER * (v - voltage);
        }
        apply();
    }

    private void apply() {
        double f = targetF();
        if (Double.isNaN(appliedF) || Math.abs(f - appliedF) >= MIN_F_CHANGE) {
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, f));
            appliedF = f;
        }
    }

    private double targetF() {
        if (!ENABLED || Double.isNaN(voltage) || voltage <= 1) return BASE_F;
        double scale = Math.max(MIN_SCALE, Math.min(MAX_SCALE, NOMINAL_VOLTAGE / voltage));
        return BASE_F * scale;
    }

    // Lowest reading across the hubs (the one the launcher is most likely on is sagging most).
    private double readVoltage() {
        double lowest = Double.NaN;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 1 && (Double.isNaN(lowest) || v < lowest)) lowest = v;
        }
        return lowest;
    }

    public double voltage() {
        return voltage;
    }

    public double f() {
        return appliedF;
    }
}
