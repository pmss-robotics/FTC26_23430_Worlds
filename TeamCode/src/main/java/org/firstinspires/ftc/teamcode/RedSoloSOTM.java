package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.PedroDriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.StateTransfer;

import java.util.Objects;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "|R|Solo|SOTM|", group = "TeleOp")
public class RedSoloSOTM extends CommandOpMode {

    public static double driveSpeed = 1;
    public static double fast       = 1;
    public static double slow       = 0.5;

    public static double offset  = 0;
    public static double targetX = 144;
    public static double targetY = 141.5;

    public static double navX       = 130.22;
    public static double navY       = 58.5;
    public static double navHeading = 34;

    private static final double OVERRIDE_THRESHOLD = 0.08;

    private boolean aimornot     = false;
    private boolean isNavigating = false;

    // Relocalization flash state
    private boolean isFlashing     = false;
    private long    flashStartTime = 0;
    private static final int  FLASH_TOTAL     = 4;
    private static final long FLASH_PERIOD_MS = 120;

    // Indicator servo positions
    private static final double IND_DEFAULT = 0.277;
    private static final double IND_YELLOW  = 0.388;
    private static final double IND_BLUE    = 0.611;
    private static final double IND_GREEN   = 0.5;

    GamepadEx driver;
    PedroDriveSubsystem drive;
    private Servo indicator;

    @Override
    public void initialize() {

        // MultipleTelemetry first so all subsystems get Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (Objects.isNull(StateTransfer.posePedro)) {
            StateTransfer.posePedro = new Pose(0, 0, Math.toRadians(0));
        }

        FlywheelSubsystem   outtake = new FlywheelSubsystem(hardwareMap, telemetry);
        IntakeSubsystemNew  intake  = new IntakeSubsystemNew(hardwareMap, telemetry);
        KickerSubsystem     kicker  = new KickerSubsystem(hardwareMap);
        HoodSubsystem       hood    = new HoodSubsystem(hardwareMap, telemetry);
        BrakeSubsystem      brake   = new BrakeSubsystem(hardwareMap);
        MovingWhileShooting turret  = new MovingWhileShooting(hardwareMap);

        turret.setInitialAngle(StateTransfer.turretInitial);

        driver = new GamepadEx(gamepad1);

        drive = new PedroDriveSubsystem(
                Constants.createFollower(hardwareMap),
                StateTransfer.posePedro,
                telemetry
        );

        hood.moveToTarget();

        indicator = hardwareMap.get(Servo.class, "indicator");
        indicator.setPosition(IND_DEFAULT);

        driveSpeed = fast;

        PedroDriveCommand driveCommand = new PedroDriveCommand(
                drive,
                () -> -driver.getLeftX() * driveSpeed,
                () ->  driver.getLeftY() * driveSpeed,
                () -> -driver.getRightX() * driveSpeed,
                true
        );

        // =========================
        // DRIVER CONTROLS
        // =========================

        // X: navigate to (navX, navY, navHeading)
        new GamepadButton(driver, GamepadKeys.Button.X).whenPressed(
                new InstantCommand(() -> {
                    Pose current = drive.getPose();
                    Pose target  = new Pose(navX, navY, Math.toRadians(navHeading));

                    PathChain navPath = drive.follower.pathBuilder()
                            .addPath(new BezierLine(current, target))
                            .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                            .build();

                    PedroDriveCommand.enabled = false;
                    drive.follower.followPath(navPath, true);
                    isNavigating = true;
                })
        );

        // Y: intake / kicker
        new GamepadButton(driver, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    kicker.moveToHome();
                    intake.setPower(1);
                }))
                .whenReleased(new InstantCommand(() -> {
                    kicker.moveToTarget();
                    aimornot = false;
                    intake.setPower(0);
                }));

        // Left Bumper: intake while held, enable flywheel on release
        new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new RunCommand(() -> {
                    intake.setPower(1);
                    aimornot = false;
                }, intake))
                .whenReleased(new InstantCommand(() -> {
                    intake.setPower(0);
                    aimornot = true;
                }));

        // D-Pad Left/Right: turret offset trim
        new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> offset += 1)
        );
        new GamepadButton(driver, GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> offset -= 1)
        );

        // D-Pad Down: pose reset + green flash confirmation
        new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> {
                    drive.setPose(new Pose(135, 9, 0));
                    isFlashing     = true;
                    flashStartTime = System.currentTimeMillis();
                })
        );

        // Right Bumper: brake
        new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(brake::moveToHome))
                .whenReleased(new RunCommand(brake::moveToTarget));

        // =========================
        // MAIN LOOP
        // =========================
        schedule(new RunCommand(() -> {

            // Navigation / manual-override
            if (isNavigating) {
                boolean driverTouching =
                        Math.abs(driver.getLeftX())  > OVERRIDE_THRESHOLD ||
                        Math.abs(driver.getLeftY())  > OVERRIDE_THRESHOLD ||
                        Math.abs(driver.getRightX()) > OVERRIDE_THRESHOLD;

                if (driverTouching || !drive.follower.isBusy()) {
                    drive.follower.breakFollowing();
                    drive.follower.startTeleopDrive();
                    PedroDriveCommand.enabled = true;
                    isNavigating = false;
                }
            }

            // Pose & velocity
            Pose   pose            = drive.getPose();
            double robotX          = pose.getX();
            double robotY          = pose.getY();
            double robotHeadingDeg = Math.toDegrees(pose.getHeading());

            Vector vel     = drive.getVelocity();
            double robotVx = vel.getXComponent();
            double robotVy = vel.getYComponent();

            // Turret — always tracking
            double aimAngle = turret.calculateAimAngleWithLead(
                    robotX, robotY, robotHeadingDeg,
                    targetX, targetY,
                    robotVx, robotVy
            );
            turret.goToAngle(aimAngle + offset);

            // Straight-line distance for hood and flywheel RPM
            double distance = Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2));

            // Hood
            double hoodPos = computeHoodPosition(distance);
            hoodPos = Math.max(0.295, Math.min(0.33, hoodPos));
            hood.setHoodPosition(hoodPos);

            // Flywheel
            if (aimornot) {
                outtake.setVelocityRpm(Math.max(1965, computeY(distance)));
            } else {
                outtake.setVelocityRpm(0);
            }

            // Aim indicator
            double turretAngle = turret.getTurretAngle();
            double error       = Math.abs((aimAngle + offset) - turretAngle);

            if (isFlashing) {
                long elapsed   = System.currentTimeMillis() - flashStartTime;
                int  halfCycle = (int)(elapsed / FLASH_PERIOD_MS);
                if (halfCycle >= FLASH_TOTAL) {
                    isFlashing = false;
                } else {
                    indicator.setPosition((halfCycle % 2 == 0) ? IND_GREEN : IND_DEFAULT);
                }
            } else if (error < 1.0) {
                indicator.setPosition(IND_BLUE);
            } else if (aimornot && error < 10.0) {
                indicator.setPosition(IND_YELLOW);
            } else {
                indicator.setPosition(IND_DEFAULT);
            }

            // Telemetry
            telemetry.addData("Navigating",         isNavigating);
            telemetry.addData("Flywheel On",         aimornot);
            telemetry.addData("Turret Angle",        turretAngle);
            telemetry.addData("Aim Angle",           aimAngle);
            telemetry.addData("Error",               error);
            telemetry.addData("Target RPM",          outtake.getTargetRPM());
            telemetry.addData("Actual RPM 1",        outtake.getActualRpm());
            telemetry.addData("Actual RPM 2",        outtake.getActualRpm2());
            telemetry.addData("Distance",            distance);
            telemetry.addData("Robot Vx",            robotVx);
            telemetry.addData("Robot Vy",            robotVy);
            telemetry.update();
        }));

        schedule(driveCommand);
    }

    public static double computeY(double x) {
        return (0.000117354  * Math.pow(x, 4))
             - (0.0384068   * Math.pow(x, 3))
             + (4.48172      * Math.pow(x, 2))
             - (207.07806     * x)
             +  5284.83659;
    }

    public static double computeHoodPosition(double x) {
        return -(1.3307e-9   * Math.pow(x, 4))
             +  (5.9712e-7   * Math.pow(x, 3))
             - (0.0000893847 * Math.pow(x, 2))
             +  (0.00474244  * x)
             +   0.250742;
    }
}
