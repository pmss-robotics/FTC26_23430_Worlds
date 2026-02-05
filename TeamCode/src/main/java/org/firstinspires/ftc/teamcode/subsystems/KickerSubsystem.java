package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class KickerSubsystem implements Subsystem {

    private Telemetry telemetry;
    private final ServoImplEx servoL;
    private final ServoImplEx servoR;

    // TODO: test HOME_POSITION & TARGET_POSITION values
    private static final double HOME_POSITION_RIGHT = 0.4;
    private static final double TARGET_POSITION_RIGHT = 0.56;
    private static final double HOME_POSITION_LEFT = 0.8;
    private static final double TARGET_POSITION_LEFT = 0.64;

    public KickerSubsystem(HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        servoL = hardwareMap.get(ServoImplEx.class, "servoL");
        servoR = hardwareMap.get(ServoImplEx.class, "servoR");

        // Initialize to home
        servoL.setPosition(HOME_POSITION_LEFT);
        servoR.setPosition(HOME_POSITION_RIGHT);
    }

    public void moveToTarget() {
        servoR.setPosition(TARGET_POSITION_RIGHT);
        servoL.setPosition(TARGET_POSITION_LEFT);
    }

    public void moveToHome() {
        servoR.setPosition(HOME_POSITION_RIGHT);
        servoL.setPosition(HOME_POSITION_LEFT);
    }

    public double getPositionLeft() {
        return servoL.getPosition();
    }

    public double getPositionRight() {
        return servoR.getPosition();
    }

    @Override
    public void periodic() {
        telemetry.addData("Kicker Left Pos", servoL.getPosition());
        telemetry.addData("Kicker Right Pos", servoR.getPosition());
    }
}