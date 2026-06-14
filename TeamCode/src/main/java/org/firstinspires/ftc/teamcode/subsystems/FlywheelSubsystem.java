package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class FlywheelSubsystem implements Subsystem {

    private final DcMotorEx motor1, motor2;
    private final Telemetry  telemetry;

    // Tunable live via FTC Dashboard.
    // kF is feedforward: target ~= 32767 / (MAX_RPM/60 * TICKS_PER_REV) ≈ 12
    // kP corrects residual error; kD damps oscillation.
    public static double kP =  340.0;
    public static double kI =   0.0;
    public static double kD =   0.0;
    public static double kF =  13.15;  // 32767 / (6000/60 * 28)

    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM       = 6000.0;

    private double targetRpm = 0.0;

    // Tracks last-applied gains so periodic() only writes when they change.
    private double lastKP, lastKI, lastKD, lastKF;

    public FlywheelSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        motor1 = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        motor2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");

        for (DcMotorEx m : new DcMotorEx[]{ motor1, motor2 }) {
            m.setDirection(DcMotorSimple.Direction.REVERSE);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        applyGains();
    }

    // --- Public API ---------------------------------------------------------

    public void setVelocityRpm(double rpm) {
        targetRpm = rpm;
        ensureEncoderMode();

        if (rpm == 0) {
            motor1.setPower(0);
            motor2.setPower(0);
            return;
        }

        double tps = rpm / 60.0 * TICKS_PER_REV;
        motor1.setVelocity(tps);
        motor2.setVelocity(tps);
    }

    /** Raw open-loop power — switches out of encoder mode. */
    public void setPower(double power) {
        targetRpm = 0;
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setPower(Range.clip(power, -1.0, 1.0));
        motor2.setPower(Range.clip(power, -1.0, 1.0));
    }

    public double getActualRpm()  { return motor1.getVelocity() * 60.0 / TICKS_PER_REV; }
    public double getActualRpm2() { return motor2.getVelocity() * 60.0 / TICKS_PER_REV; }
    public double getTargetRPM()  { return targetRpm; }

    // --- Subsystem loop -----------------------------------------------------

    @Override
    public void periodic() {
        if (kP != lastKP || kI != lastKI || kD != lastKD || kF != lastKF) {
            applyGains();
        }
        telemetry.addData("Target RPM",   targetRpm);
        telemetry.addData("Actual RPM 1", getActualRpm());
        telemetry.addData("Actual RPM 2", getActualRpm2());
    }

    // --- Helpers ------------------------------------------------------------

    private void applyGains() {
        PIDFCoefficients c = new PIDFCoefficients(kP, kI, kD, kF);
        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, c);
        lastKP = kP; lastKI = kI; lastKD = kD; lastKF = kF;
    }

    private void ensureEncoderMode() {
        if (motor1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            applyGains();
        }
    }
}
