package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.seattlesolvers.solverslib.command.SubsystemBase;

/** Plain robot-centric mecanum drive. No odometry. */
public class MecanumDriveSubsystem extends SubsystemBase {
    private final DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private final GoBildaPinpointDriver pinpoint;

    public MecanumDriveSubsystem(HardwareMap hw) {
        leftFront = hw.get(DcMotorEx.class, "leftFront");
        leftBack = hw.get(DcMotorEx.class, "leftBack");
        rightFront = hw.get(DcMotorEx.class, "rightFront");
        rightBack = hw.get(DcMotorEx.class, "rightBack");

        // Pinpoint is optional: without it, field-centric falls back to robot-centric
        GoBildaPinpointDriver pp = null;
        try {
            pp = hw.get(GoBildaPinpointDriver.class, "pinpoint");
            pp.resetPosAndIMU(); // robot must be still; current facing becomes heading 0
        } catch (Exception e) {
            pp = null;
        }
        pinpoint = pp;

        // Flip these if a side drives backwards
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /** forward: +up on stick, strafe: +right, turn: +clockwise. All -1..1. */
    public void drive(double forward, double strafe, double turn) {
        double lf = forward + strafe + turn;
        double lb = forward - strafe + turn;
        double rf = forward - strafe - turn;
        double rb = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                Math.max(Math.abs(rf), Math.abs(rb))));

        leftFront.setPower(lf / max);
        leftBack.setPower(lb / max);
        rightFront.setPower(rf / max);
        rightBack.setPower(rb / max);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    /** Same inputs as drive(), but forward is always away from the driver (field forward). */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        double h = getHeading();
        double cos = Math.cos(h), sin = Math.sin(h);
        // Rotate the field-frame stick vector into the robot frame
        double robotForward = forward * cos - strafe * sin;
        double robotStrafe = forward * sin + strafe * cos;
        drive(robotForward, robotStrafe, turn);
    }

    /** Heading in radians, CCW positive. */
    public double getHeading() {
        return pinpoint == null ? 0 : pinpoint.getHeading(AngleUnit.RADIANS);
    }

    /** Makes the way the robot currently faces the new field forward. */
    public void resetHeading() {
        if (pinpoint != null) pinpoint.resetPosAndIMU();
    }

    @Override
    public void periodic() {
        if (pinpoint != null) pinpoint.update();
    }
}
