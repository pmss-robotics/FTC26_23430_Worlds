package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/** Two intake motors that run together as one unit. */
public class IntakeSubsystem extends SubsystemBase {
    // Hardware config names, change to match your robot configuration
    private static final String MOTOR_1_NAME = "intake";
    private static final String MOTOR_2_NAME = "intake2";

    // Flip a direction if that motor pulls balls the wrong way
    private static final DcMotorSimple.Direction MOTOR_1_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction MOTOR_2_DIR = DcMotorSimple.Direction.REVERSE;

    private final DcMotorEx motor1, motor2;
    private double power = 0;

    public IntakeSubsystem(HardwareMap hw) {
        motor1 = hw.get(DcMotorEx.class, MOTOR_1_NAME);
        motor2 = hw.get(DcMotorEx.class, MOTOR_2_NAME);
        motor1.setDirection(MOTOR_1_DIR);
        motor2.setDirection(MOTOR_2_DIR);
        for (DcMotorEx m : new DcMotorEx[]{motor1, motor2}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /** -1..1. Positive pulls balls in. */
    public void setPower(double p) {
        power = Range.clip(p, -1.0, 1.0);
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public void on() { setPower(1.0); }

    public void reverse() { setPower(-1.0); }

    public void off() { setPower(0); }

    public double getPower() { return power; }
}
