package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.PedroDriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BrakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystemNew;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MovingWhileShooting;
import org.firstinspires.ftc.teamcode.subsystems.PedroDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.StateTransfer;

import java.util.Objects;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "|B|Solo|SOTMv2|", group = "TeleOp")
public class BlueSoloSOTMAutomated extends CommandOpMode {

    public static double driveSpeed = 1;
    public static double fast       = 1;
    public static double slow       = 0.5;

    public static double offset  = 0;
    public static double targetX = 144;
    public static double targetY = 0;
    private double smoothedRPM = 0;

    public static double navX       = 130.22;
    public static double navY       = 144 - 58.5;
    public static double navHeading = -34;

    private static final double OVERRIDE_THRESHOLD = 0.08;

    private boolean aimornot     = false;
    private boolean autoShoot    = false;
    private boolean isNavigating  = false;

// BURST FIRE MACRO
public static int BURST_COUNT = 3;
    private int burstShotsRemaining = 0;
    private long kickerTimer = 0;

    private enum KickerState { IDLE, FIRING, RETRACTING }
    private KickerState kickerState = KickerState.IDLE;

    // Timing constants in milliseconds
    public static long FIRE_TIME_MS = 150;
    public static long RETRACT_TIME_MS = 250;

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
    private VoltageSensor batteryVoltageSensor;

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
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

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

        // Y: intake / automated kicker
        new GamepadButton(driver, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    kicker.moveToHome();
                    intake.setPower(1);
                    aimornot = false;
                    autoShoot = false;
                    burstShotsRemaining = 0;
                }))
                .whenReleased(new InstantCommand(() -> {
                    intake.setPower(0);
                }));

        // Left Bumper: intake while held, automatically prepare to shoot on release
        new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new RunCommand(() -> {
                    intake.setPower(1);
                    aimornot = false;
                    burstShotsRemaining = 0;
                    kickerState = KickerState.IDLE;
                }, intake))
                .whenReleased(new InstantCommand(() -> {
                    intake.setPower(0);
                    aimornot = true;
                    burstShotsRemaining = BURST_COUNT; // Queues 3 shots
                }));

        // D-Pad Left/Right: turret offset trim
        new GamepadButton(driver, GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> offset = Math.min(10, offset + 1))
        );
        new GamepadButton(driver, GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(() -> offset = Math.max(-10,offset - 1))
        );

        // D-Pad Down: pose reset + green flash confirmation
        new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> {
                    drive.setPose(new Pose(135, 141.5 - 9, 0));
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
            driver.readButtons();

            // Define driver override status
            boolean driverTouching =
                    Math.abs(driver.getLeftX())  > OVERRIDE_THRESHOLD ||
                            Math.abs(driver.getLeftY())  > OVERRIDE_THRESHOLD ||
                            Math.abs(driver.getRightX()) > OVERRIDE_THRESHOLD;

            // Navigation / manual-override
            if (isNavigating) {
                if (driverTouching) {
                    // Driver took over early: Cancel navigation, do NOT brake
                    drive.follower.breakFollowing();
                    drive.follower.startTeleopDrive();
                    PedroDriveCommand.enabled = true;
                    isNavigating = false;

                } else if (!drive.follower.isBusy()) {
                    // Path finished successfully: Cancel navigation AND auto-brake
                    drive.follower.breakFollowing();
                    drive.follower.startTeleopDrive();
                    PedroDriveCommand.enabled = true;
                    isNavigating = false;

                    brake.moveToHome(); // Engage the brake automatically
                }
            } else {
                // Auto-Release: If no longer navigating, and the driver moves
                // the sticks to drive away, automatically release the brakes
                // (unless they are manually holding brake).
                if (driverTouching && !gamepad1.right_bumper) {
                    brake.moveToTarget();
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
                // Read current Control Hub battery voltage
                double batteryVoltage = batteryVoltageSensor.getVoltage();
                double targetRPM = Math.max(1965, computeY(distance));

                // only compensate voltage when under 12.8
                double voltageCompensation = 1.0;

                if (batteryVoltage < 12.7) {
                    voltageCompensation = 12.7 / Math.max(batteryVoltage, 10.0);
                }
                targetRPM *= voltageCompensation;

                smoothedRPM += (targetRPM - smoothedRPM) * 0.10;
                outtake.setVelocityRpm(smoothedRPM);
            } else {
                outtake.setVelocityRpm(0);
            }

// Shooting readiness
            double turretAngle = turret.getTurretAngle();
            double error = Math.abs((aimAngle + offset) - turretAngle);

            double rpmError = Math.abs(outtake.getActualRpm() - outtake.getTargetRPM());

            boolean turretReady = error < 1.0;
            boolean flywheelReady = rpmError < 50;
            boolean shootReady = turretReady && flywheelReady;

// Automatic shooting
            /*if (autoShoot && shootReady) {
                kicker.moveToTarget();
                autoShoot = false;
            }*/

            // Burst Fire State Machine
            if (burstShotsRemaining > 0) {
                switch (kickerState) {
                    case IDLE:
                        if (shootReady) {
                            kicker.moveToTarget();
                            kickerTimer = System.currentTimeMillis();
                            kickerState = KickerState.FIRING;
                        }
                        break;
                    case FIRING:
                        if (System.currentTimeMillis() - kickerTimer > FIRE_TIME_MS) {
                            kicker.moveToHome();
                            kickerTimer = System.currentTimeMillis();
                            kickerState = KickerState.RETRACTING;
                            burstShotsRemaining--;
                        }
                        break;
                    case RETRACTING:
                        if (System.currentTimeMillis() - kickerTimer > RETRACT_TIME_MS) {
                            kickerState = KickerState.IDLE;
                        }
                        break;
                }
            } else {
                kickerState = KickerState.IDLE;
            }


            if (isFlashing) {
                long elapsed   = System.currentTimeMillis() - flashStartTime;
                int  halfCycle = (int)(elapsed / FLASH_PERIOD_MS);
                if (halfCycle >= FLASH_TOTAL) {
                    isFlashing = false;
                } else {
                    indicator.setPosition((halfCycle % 2 == 0) ? IND_GREEN : IND_DEFAULT);
                }
            } else if (shootReady) {
                indicator.setPosition(IND_BLUE);
            } else if (aimornot && error < 10.0) {
                indicator.setPosition(IND_YELLOW);
            } else {
                indicator.setPosition(IND_DEFAULT);
            }

            // Telemetry
            telemetry.addData("Navigating",         isNavigating);
            telemetry.addData("Flywheel On",         aimornot);
            telemetry.addData("Burst Shots Left",   burstShotsRemaining);
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
        return (0.000057964  * Math.pow(x, 4))
                - (0.0197251    * Math.pow(x, 3))
                + (2.38515      * Math.pow(x, 2))
                - (107.9964     * x)
                +  3705.22859;
    }

    public static double computeHoodPosition(double x) {
        return -(1.3307e-9   * Math.pow(x, 4))
                +  (5.9712e-7   * Math.pow(x, 3))
                - (0.0000893847 * Math.pow(x, 2))
                +  (0.00474244  * x)
                +   0.250742;
    }
}
