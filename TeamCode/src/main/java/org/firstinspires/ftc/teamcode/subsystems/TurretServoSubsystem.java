package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/**
 * Two Axon servos (servo mode) that move together as one unit, e.g. a geared turret.
 * Both servos are always commanded to the same logical position, so they turn the same way.
 * Position is 0.0 to 1.0 across the Axon's full range (about 355 degrees).
 */
public class TurretServoSubsystem extends SubsystemBase {
    // Hardware config names, change to match your robot configuration
    private static final String LEFT_NAME = "turretServo1";
    private static final String RIGHT_NAME = "turretServo2";

    // Axon full PWM range
    private static final double PWM_MIN_US = 500;
    private static final double PWM_MAX_US = 2500;

    // Software limits so the turret can't be driven past its mechanical stops
    private static final double MIN_POSITION = 0.0;
    private static final double MAX_POSITION = 1.0;

    // Position the turret is held at on init (0.5 = center)
    private static final double START_POSITION = 0.5;

    // Position change per second at full joystick deflection
    private static final double MAX_SPEED_PER_SEC = 1.8;
    private static final double DEADBAND = 0.05;

    private final ServoImplEx servoA;
    private final ServoImplEx servoB;

    private double position = START_POSITION;
    private long lastNanos = System.nanoTime();

    public TurretServoSubsystem(HardwareMap hw) {
        servoA = hw.get(ServoImplEx.class, LEFT_NAME);
        servoB = hw.get(ServoImplEx.class, RIGHT_NAME);

        servoA.setPwmRange(new PwmControl.PwmRange(PWM_MIN_US, PWM_MAX_US));
        servoB.setPwmRange(new PwmControl.PwmRange(PWM_MIN_US, PWM_MAX_US));

        // If the servos face each other and one ends up spinning the opposite way,
        // flip this one to REVERSE. Leave both FORWARD if they're mounted the same way.
        servoA.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.FORWARD);
        servoB.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);

        setPosition(START_POSITION);
    }

    /** Jump to an absolute position (0 to 1). */
    public void setPosition(double pos) {
        position = Range.clip(pos, MIN_POSITION, MAX_POSITION);
        servoA.setPosition(position);
        servoB.setPosition(position);
    }

    /**
     * Joystick control: input in [-1, 1] sets how fast the position moves.
     * Call once per loop. Releasing the stick holds the current position.
     */
    public void drive(double input) {
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        lastNanos = now;

        if (Math.abs(input) < DEADBAND) return;
        setPosition(position + input * MAX_SPEED_PER_SEC * dt);
    }

    public double getPosition() {
        return position;
    }

    /** Servo mode holds position on its own; this just keeps the dt timer fresh when re-enabled. */
    public void resetTimer() {
        lastNanos = System.nanoTime();
    }
}
