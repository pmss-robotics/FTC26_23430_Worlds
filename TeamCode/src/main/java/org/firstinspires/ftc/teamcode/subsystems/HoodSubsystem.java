package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HoodSubsystem implements Subsystem {

    Telemetry telemetry;
    ServoImplEx servoHood;

    private double HOME_POSITION = 0.4;
    private double TARGET_POSITION = 0.32;

    public HoodSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        servoHood = hardwareMap.get(ServoImplEx.class, "hood");
        servoHood.setPosition(TARGET_POSITION);
    }

    public void moveToTarget() {
        servoHood.setPosition(TARGET_POSITION);
    }

    public void moveToHome() {
        servoHood.setPosition(HOME_POSITION);
    }

    public void setTARGET_POSITION(boolean positive){
        if (positive) {
            this.TARGET_POSITION += 0.01;
        } else {
            this.TARGET_POSITION -= 0.01;
        }
    }

    public void setHOME_POSITION() {
        this.HOME_POSITION += 0.05;
    }

    public double getTARGET_POSITION() {
        return this.TARGET_POSITION;
    }

    // ⭐ NEW METHOD — allows TeleOp to directly set hood servo position
    public void setHoodPosition(double pos) {
        servoHood.setPosition(pos);
    }
}
