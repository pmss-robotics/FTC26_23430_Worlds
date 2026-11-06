package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/** Single-motor velocity-controlled flywheel. Gains are tunable live via FTC Dashboard. */
@Config
public class FlywheelSubsystem extends SubsystemBase {
    // Hardware config name, change to match your robot configuration
    private static final String MOTOR_NAME = "flywheel";

    // Same gains as last year. kF ~= 32767 / (MAX_RPM/60 * TICKS_PER_REV)
    public static double kP = 340.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 13.15;

    // Change to match your motor (28 / 6000 is a 6000 rpm goBILDA Yellow Jacket bare motor)
    private static final double TICKS_PER_REV = 28.0;

    // A shot counts as ready when within this many rpm of the target
    private static final double READY_TOLERANCE_RPM = 75;

    private final DcMotorEx motor;
    private double targetRpm = 0;
    private double lastKP, lastKI, lastKD, lastKF;

    public FlywheelSubsystem(HardwareMap hw) {
        motor = hw.get(DcMotorEx.class, MOTOR_NAME);
        // Flip to FORWARD if the wheel spins the wrong way
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        applyGains();
    }

    public void setVelocityRpm(double rpm) {
        targetRpm = Math.max(0, rpm);
        if (targetRpm == 0) {
            motor.setPower(0);
        } else {
            motor.setVelocity(targetRpm / 60.0 * TICKS_PER_REV);
        }
    }

    public void stop() {
        setVelocityRpm(0);
    }

    public double getTargetRpm() { return targetRpm; }

    public double getActualRpm() { return motor.getVelocity() * 60.0 / TICKS_PER_REV; }

    public boolean atSpeed() {
        return targetRpm > 0 && Math.abs(getActualRpm() - targetRpm) < READY_TOLERANCE_RPM;
    }

    @Override
    public void periodic() {
        if (kP != lastKP || kI != lastKI || kD != lastKD || kF != lastKF) applyGains();
    }

    private void applyGains() {
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, kI, kD, kF));
        lastKP = kP; lastKI = kI; lastKD = kD; lastKF = kF;
    }
}
