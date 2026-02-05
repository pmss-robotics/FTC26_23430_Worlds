package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import com.seattlesolvers.solverslib.command.Subsystem;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.States;

public class BeltSubsystem implements Subsystem {

    private final Telemetry telemetry;
    private final DcMotorEx beltMotor;

    private States.OuttakeExtension currentOuttakeState;

    public BeltSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.beltMotor = hardwareMap.get(DcMotorEx.class, "transfer");

        currentOuttakeState = States.OuttakeExtension.home;
    }

    public States.OuttakeExtension getCurrentOuttakeState() {
        return currentOuttakeState;
    }

    public void setOuttakeState(States.OuttakeExtension state) {
        currentOuttakeState = state;
    }

    public void setPower(double power) {
        double clippedPower = Range.clip(power, -1.0, 1.0);
        beltMotor.setPower(clippedPower);
    }

    @Override
    public void periodic() {
        telemetry.addData("Belt State", currentOuttakeState);
        telemetry.addData("Belt Power", beltMotor.getPower());
    }
}