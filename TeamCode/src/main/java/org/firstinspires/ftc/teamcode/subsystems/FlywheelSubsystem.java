package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class FlywheelSubsystem implements Subsystem {

    private final Telemetry telemetry;
    private DcMotorEx outtakeMotor;
    private DcMotorEx outtakeMotor2;

    // PIDF coefficients
    private static final double kP = 400, kI = 0.0, kD = 0.0;
    private static final double TICKS_PER_REV = 28.0;
    private static final double MAX_RPM = 6000.0;
    private static final double kF = 13.106;
    //private static final double kF = 14.47;

    private double targetRpm = 0.0;

    public FlywheelSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setVelocityPIDFCoefficients(kP, kI, kD, kF);

        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtakeMotor2");
        outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor2.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    public void setPower(double power) {
        outtakeMotor.setPower(Range.clip(power, -1.0, 1.0));
        outtakeMotor2.setPower(Range.clip(power, -1.0, 1.0));
        targetRpm = 0.0;
    }

    public void setVelocityRpm(double rpm) {
        targetRpm = rpm;
        double ticksPerSec = (rpm * TICKS_PER_REV) / 60.0;
        outtakeMotor.setVelocity(ticksPerSec);
        outtakeMotor2.setVelocity(ticksPerSec);
    }

    public double getActualRpm(){
        return (outtakeMotor.getVelocity() * 60.0)/TICKS_PER_REV;
    }

    public double getActualRpm2(){
        return (outtakeMotor2.getVelocity() * 60.0)/TICKS_PER_REV;
    }

    public double getTargetRPM(){
        return targetRpm;
    }

    @Override
    public void periodic() {
        double actualRpm = (outtakeMotor.getVelocity() * 60.0) / TICKS_PER_REV;
        double actualRpm2 = (outtakeMotor2.getVelocity() * 60.0) / TICKS_PER_REV;

        telemetry.addData("Target RPM", targetRpm);
        telemetry.addData("Actual RPM 1", actualRpm);
        telemetry.addData("Actual RPM 2", actualRpm);
    }
}