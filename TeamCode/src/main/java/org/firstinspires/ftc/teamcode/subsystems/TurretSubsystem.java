package org.firstinspires.ftc.teamcode.subsystems;

import static com.seattlesolvers.solverslib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.seattlesolvers.solverslib.command.Subsystem;

@Config
public class TurretSubsystem implements Subsystem {

    private final DcMotor turretMotor;

    public static double TICKS_PER_REV = 526.54;

    // PID coefficients
    public static double kP = 0.022;   // UPDATED
    public static double kI = 0.0;
    public static double kD = 0.005;

    // Safe rotation limits
    public static double MIN_ANGLE = -135;   // right
    public static double MAX_ANGLE = 135;    // left

    // Hold-zero mode
    public boolean holdZero = false;

    private double integral = 0;
    private double lastError = 0;

    private double dx = 0;
    private double dy =0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turret");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Positive = LEFT, Negative = RIGHT
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getTurretAngle() {
        double angle = (turretMotor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
        return normalizeAngle(angle);
    }

    private double normalizeAngle(double angle) {
        angle %= 360.0;
        if (angle > 180.0) angle -= 360.0;
        if (angle < -180.0) angle += 360.0;
        return angle;
    }

    public double calculateAimAngle(double robotX, double robotY, double robotHeadingDeg,
                                    double targetX, double targetY) {

        dx = targetX - robotX;
        dy = targetY - robotY;

        double angleToTarget = Math.toDegrees(Math.atan2(dy, dx));

        double turretAngle = angleToTarget - robotHeadingDeg;

        return normalizeAngle(turretAngle);
    }

    public double getDistance(){
        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
    }

    public void holdAtZero(boolean hold) {
        holdZero = hold;
    }

    public void goToAngle(double targetAngle) {

        // Override with hold-zero
        if (holdZero) {
            targetAngle = 0;
        }

        // Clamp target
        targetAngle = clamp(targetAngle, MIN_ANGLE, MAX_ANGLE);

        double current = getTurretAngle();
        double error = normalizeAngle(targetAngle - current);

        // HARD STOP protection
        if (current >= MAX_ANGLE && error > 0) {
            turretMotor.setPower(0);
            return;
        }
        if (current <= MIN_ANGLE && error < 0) {
            turretMotor.setPower(0);
            return;
        }

        // PID
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double output = (kP * error) + (kI * integral) + (kD * derivative);

        // Clamp power
        output = clamp(output, -1, 1);

        turretMotor.setPower(output);
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}