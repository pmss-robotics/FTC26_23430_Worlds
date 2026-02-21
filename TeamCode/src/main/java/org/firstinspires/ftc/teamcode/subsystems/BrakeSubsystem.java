package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.seattlesolvers.solverslib.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BrakeSubsystem implements Subsystem {

    private Telemetry telemetry;
    //private final ServoImplEx brakeL;
    private final ServoImplEx brakeR;

    // TODO: test HOME_POSITION & TARGET_POSITION values
    private static final double HOME_POSITION_RIGHT = 0;
    private static final double TARGET_POSITION_RIGHT = 0.42;
    private static final double HOME_POSITION_LEFT = 1;
    private static final double TARGET_POSITION_LEFT = 0;

    public BrakeSubsystem(HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        //brakeL = hardwareMap.get(ServoImplEx.class, "brakeL");
        brakeR = hardwareMap.get(ServoImplEx.class, "brakeR");

        // Initialize to home
        //brakeL.setPosition(TARGET_POSITION_LEFT);
        brakeR.setPosition(TARGET_POSITION_RIGHT);
    }

    public void moveToTarget() {
        //brakeL.setPosition(TARGET_POSITION_RIGHT);
        brakeR.setPosition(TARGET_POSITION_RIGHT);
    }

    public void moveToHome() {
        //brakeL.setPosition(HOME_POSITION_RIGHT);
        brakeR.setPosition(HOME_POSITION_RIGHT);
    }

    //public double getPositionLeft() {
    //    return brakeL.getPosition();
    //}

    public double getPositionRight() {
        return brakeR.getPosition();
    }

    @Override
    public void periodic() {
        //telemetry.addData("Brake Left Pos", brakeL.getPosition());
        telemetry.addData("Brake Right Pos", brakeR.getPosition());
    }
}