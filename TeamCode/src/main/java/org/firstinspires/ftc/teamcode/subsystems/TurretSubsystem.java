package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.util.StateTransfer.turretInitial;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Subsystem;
import org.firstinspires.ftc.teamcode.util.StateTransfer;
@Config
public class TurretSubsystem implements Subsystem {

    private final DcMotor turretMotor;

    public static double TICKS_PER_REV = 526.54;

    // PID coefficients
    public static double kP = 0.075;
    public static double kI = 0.0;
    public static double kD = 0.003;

    // Safe rotation limits
    public static double MIN_ANGLE = -150;
    public static double MAX_ANGLE = 150;

    // Desired startup angle
    public static double INITIAL_ANGLE = turretInitial;   // <<< CHANGE THIS

    // Hold-zero mode
    public boolean holdZero = false;

    private double integral = 0;
    private double lastError = 0;

    private double dx = 0;
    private double dy = 0;

    // Angle offset
    private double angleOffset = 0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turret");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Apply initial angle offset
        setInitialAngle(INITIAL_ANGLE);
    }

    // Raw encoder angle (no offset)
    private double getRawAngle() {
        return (turretMotor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
    }

    // Public angle with offset applied
    public double getTurretAngle() {
        return normalizeAngle(getRawAngle() + angleOffset);
    }

    // Normalize to [-180, 180]
    private double normalizeAngle(double angle) {
        angle %= 360.0;
        if (angle > 180.0) angle -= 360.0;
        if (angle < -180.0) angle += 360.0;
        return angle;
    }

    // Set the turret's "starting angle"
    public void setInitialAngle(double angleDeg) {
        angleOffset = angleDeg - getRawAngle();
    }

    public double calculateAimAngle(double robotX, double robotY, double robotHeadingDeg,
                                    double targetX, double targetY) {

        dx = targetX - robotX;
        dy = targetY - robotY;

        double angleToTarget = Math.toDegrees(Math.atan2(dy, dx));
        double turretAngle = angleToTarget - robotHeadingDeg;

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

        targetAngle = Range.clip(targetAngle, MIN_ANGLE, MAX_ANGLE);

        double current = getTurretAngle();
        double error = normalizeAngle(targetAngle - current);

        // Hard stop protection
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

        output = Range.clip(output, -0.8, 0.8);

        turretMotor.setPower(output);
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}
