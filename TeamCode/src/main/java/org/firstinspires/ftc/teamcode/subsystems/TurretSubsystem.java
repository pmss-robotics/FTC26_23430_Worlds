package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.StateTransfer.turretInitial;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Subsystem;

@Config
public class TurretSubsystem implements Subsystem {

    private final DcMotor turretMotor;

    public static double TICKS_PER_REV = 1792.3333333333;

    public static double kP = 0.038;
    public static double kI = 0.0;
    public static double kD = 0.002;

    public static double MAX_LEFT  =  120.0;
    public static double MAX_RIGHT = -240.0;

    public static double INITIAL_ANGLE = turretInitial;

    public boolean holdZero = false;

    private double integral  = 0;
    private double lastError = 0;

    private double dx = 0;
    private double dy = 0;

    private double angleOffset = 0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turret");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setInitialAngle(INITIAL_ANGLE);
    }

    private double getRawAngle() {
        return (turretMotor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
    }

    public double getTurretAngle() {
        return getRawAngle() + angleOffset;
    }

    public void setInitialAngle(double angleDeg) {
        angleOffset = angleDeg - getRawAngle();
    }

    private double normalizeAngle(double angle) {
        angle %= 360.0;
        if (angle > 180.0)  angle -= 360.0;
        if (angle < -180.0) angle += 360.0;
        return angle;
    }

    // Find the copy of normalizedTarget (within ±360 of nearest) that is
    // actually inside [MAX_RIGHT, MAX_LEFT] and closest to current
    private double resolveTarget(double normalizedTarget, double current) {
        double base = normalizedTarget
                + Math.round((current - normalizedTarget) / 360.0) * 360.0;

        double best = Double.MAX_VALUE;
        double bestDist = Double.MAX_VALUE;

        for (double candidate : new double[]{ base - 360.0, base, base + 360.0 }) {
            if (candidate >= MAX_RIGHT - 1e-6 && candidate <= MAX_LEFT + 1e-6) {
                double dist = Math.abs(candidate - current);
                if (dist < bestDist) {
                    bestDist = dist;
                    best = candidate;
                }
            }
        }

        // If no candidate fell in range, clip the base to the nearest limit
        if (best == Double.MAX_VALUE) {
            best = Range.clip(base, MAX_RIGHT, MAX_LEFT);
        }

        return best;
    }

    public double calculateAimAngle(double robotX, double robotY, double robotHeadingDeg,
                                    double targetX, double targetY) {
        dx = targetX - robotX;
        dy = targetY - robotY;

        double angleToTarget = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngle   = angleToTarget - robotHeadingDeg;

        return normalizeAngle(turretAngle);
    }

    public double getDistance() {
        return Math.sqrt(dx * dx + dy * dy);
    }

    public void holdAtZero(boolean hold) {
        holdZero = hold;
    }

    public void goToAngle(double targetAngle) {

        if (holdZero) targetAngle = 0;

        targetAngle = normalizeAngle(targetAngle);

        double current  = getTurretAngle();
        double resolved = resolveTarget(targetAngle, current);
        double error    = resolved - current;

        // Hard stops
        if (current >= MAX_LEFT && error > 0) {
            turretMotor.setPower(0);
            integral = 0;
            return;
        }
        if (current <= MAX_RIGHT && error < 0) {
            turretMotor.setPower(0);
            integral = 0;
            return;
        }

        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double output = (kP * error) + (kI * integral) + (kD * derivative);
        output = Range.clip(output, -1.0, 1.0);

        turretMotor.setPower(output);
    }

    public void stop() {
        turretMotor.setPower(0);
        integral = 0;
    }
}