package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // Set to 1.0 or -1.0 depending on which direction pulls balls in
    private static final double INTAKE_POWER = 1.0;

    private final DcMotorEx intake;

    public IntakeSubsystem(HardwareMap hw) {
        intake = hw.get(DcMotorEx.class, "intake");
    }

    public void on() {
        intake.setPower(INTAKE_POWER);
    }

    public void off() {
        intake.setPower(0);
    }
}
