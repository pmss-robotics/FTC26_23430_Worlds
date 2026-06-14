package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.StateTransfer.turretInitial;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Subsystem;

@Config
public class MovingWhileShooting implements Subsystem {

    private final DcMotor turretMotor;

    public static double  TICKS_PER_REV = 1792.3333333333;
    public static boolean INVERT_MOTOR  = false; // flip from Dashboard if turret tracks wrong direction

    public static double kP = 0.038;
    public static double kI = 0.0;
    public static double kD = 0.002;

    public static double MAX_LEFT  =  120.0;
    public static double MAX_RIGHT = -240.0;

    // The only value to tune for lead compensation — your projectile speed in inches/sec
    public static double PROJECTILE_SPEED = 144.0;

    private double angleOffset = 0;
    private double integral    = 0;
    private double lastError   = 0;

    public MovingWhileShooting(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setInitialAngle(turretInitial);
    }

    // ── Angle reading ──────────────────────────────────────────────────────────

    private double getRawAngle() {
        double ticks = turretMotor.getCurrentPosition();
        if (INVERT_MOTOR) ticks = -ticks;
        return (ticks / TICKS_PER_REV) * 360.0;
    }

    public double getTurretAngle() {
        return getRawAngle() + angleOffset;
    }

    public void setInitialAngle(double angleDeg) {
        angleOffset = angleDeg - getRawAngle();
    }

    // ── Lead calculation ───────────────────────────────────────────────────────
    //
    // Physics: the ball carries the robot's velocity when fired.
    // For the ball to reach a fixed target:
    //   aim_vec = (targetX - robotX - Vx*t,  targetY - robotY - Vy*t)
    //   t       = |aim_vec| / PROJECTILE_SPEED   (two passes to converge)
    //
    // Tune PROJECTILE_SPEED until static shots land correctly,
    // then moving shots automatically compensate.

    public double calculateAimAngleWithLead(double robotX, double robotY, double robotHeadingDeg,
                                            double targetX, double targetY,
                                            double robotVx, double robotVy) {
        double baseDx = targetX - robotX;
        double baseDy = targetY - robotY;
        double dx     = baseDx;
        double dy     = baseDy;

        if (PROJECTILE_SPEED > 1e-3) {
            // First pass: estimate flight time from current straight-line distance
            double t = Math.sqrt(baseDx * baseDx + baseDy * baseDy) / PROJECTILE_SPEED;
            dx = baseDx - robotVx * t;
            dy = baseDy - robotVy * t;

            // Second pass: refine using the lead-corrected distance
            t  = Math.sqrt(dx * dx + dy * dy) / PROJECTILE_SPEED;
            dx = baseDx - robotVx * t;
            dy = baseDy - robotVy * t;
        }

        return normalize(Math.toDegrees(Math.atan2(dy, dx)) - robotHeadingDeg);
    }

    // ── PID tracking ───────────────────────────────────────────────────────────

    public void goToAngle(double targetAngleDeg) {
        double current  = getTurretAngle();
        double resolved = resolveTarget(normalize(targetAngleDeg), current);
        double error    = resolved - current;

        // Hard stops — cut power and clear integral at limits
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

        integral  = Range.clip(integral + error, -50.0, 50.0);
        double derivative = error - lastError;
        lastError = error;

        double output = kP * error + kI * integral + kD * derivative;
        if (INVERT_MOTOR) output = -output;
        turretMotor.setPower(Range.clip(output, -1.0, 1.0));
    }

    public void stop() {
        turretMotor.setPower(0);
        integral  = 0;
        lastError = 0;
    }

    // ── Helpers ────────────────────────────────────────────────────────────────

    private double normalize(double angle) {
        angle %= 360.0;
        if (angle >  180.0) angle -= 360.0;
        if (angle < -180.0) angle += 360.0;
        return angle;
    }

    private double resolveTarget(double normalizedTarget, double current) {
        double base     = normalizedTarget
                        + Math.round((current - normalizedTarget) / 360.0) * 360.0;
        double best     = base;
        double bestDist = Double.MAX_VALUE;

        for (double candidate : new double[]{ base - 360.0, base, base + 360.0 }) {
            if (candidate >= MAX_RIGHT - 1e-6 && candidate <= MAX_LEFT + 1e-6) {
                double dist = Math.abs(candidate - current);
                if (dist < bestDist) {
                    bestDist = dist;
                    best     = candidate;
                }
            }
        }

        if (bestDist == Double.MAX_VALUE) {
            best = Range.clip(base, MAX_RIGHT, MAX_LEFT);
        }

        return best;
    }
}
