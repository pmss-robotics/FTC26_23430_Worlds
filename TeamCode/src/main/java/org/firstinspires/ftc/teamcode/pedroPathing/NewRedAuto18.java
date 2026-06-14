package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.StateTransfer;

@Autonomous(name = "|R|18|N|", group = "Autonomous")
public class NewRedAuto18 extends OpMode {
    KickerSubsystem kicker;
    private Servo indicator;
    IntakeSubsystemNew intake;
    HoodSubsystem hood;
    FlywheelSubsystem outtake;
    MovingWhileShooting turret;
    private boolean aimornot = false;
    public static double targetX = 144;
    public static double targetY = 144;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // Path1: starting movement (single Path, not PathChain, for setStartingPose consistency)
    private Path Path1;
    private PathChain Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, Path11;

    @Override
    public void init() {
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystemNew(hardwareMap, telemetry);
        hood = new HoodSubsystem(hardwareMap, telemetry);
        outtake = new FlywheelSubsystem(hardwareMap, telemetry);
        turret = new MovingWhileShooting(hardwareMap);
        indicator = hardwareMap.get(Servo.class, "indicator");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        follower.setStartingPose(new Pose(128.000, 111.000, Math.toRadians(0)));
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        Pose   pose            = follower.getPose();
        double robotX          = pose.getX();
        double robotY          = pose.getY();
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        Vector vel     = follower.getVelocity();
        double robotVx = vel.getXComponent();
        double robotVy = vel.getYComponent();

        double distance = Math.sqrt(Math.pow(targetX - robotX, 2) + Math.pow(targetY - robotY, 2));

        double hoodPos = computeHoodPosition(distance);
        hood.setHoodPosition(Math.max(0.295, Math.min(0.33, hoodPos)));

        if (aimornot) {
            outtake.setVelocityRpm(Math.max(1965, computeY(distance)));
        } else {
            outtake.setVelocityRpm(0);
        }

        double aimAngle = turret.calculateAimAngleWithLead(
                robotX, robotY, robotHeadingDeg, targetX, targetY, robotVx, robotVy);
        turret.goToAngle(aimAngle);

        StateTransfer.turretInitial = turret.getTurretAngle();
        indicator.setPosition(Math.abs(aimAngle - turret.getTurretAngle()) < 2 ? 0.611 : 0.277);

        telemetry.addData("path state", pathState);
        telemetry.update();
    }

    @Override
    public void stop() {
        StateTransfer.posePedro = follower.getPose();
    }

    public void buildPaths() {

        // Path 1: Starting position (125,110) → intake zone (95,83)
        // Curved approach — heading interpolates from 0° to -30°
        Path1 = new Path(new BezierCurve(
                new Pose(128.000, 111.000),
                new Pose(102.000, 103.000),
                new Pose(91.000, 76.000)));
        Path1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30));

        // Path 2: Intake zone (95,83) → first shoot spot (118,59)
        // Curved — heading from -30° to 0°
        Path2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(91.000, 76.000),
                                new Pose(101.000, 58.000),
                                new Pose(123.000, 59.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(123.000, 59.000),
                                new Pose(91.000, 76.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-38))

                .build();

        Path4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(91.000, 76.000),
                                new Pose(111.000, 49.000),
                                new Pose(133.000, 60.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(133.000, 60.000),
                                new Pose(111.000, 49.000),
                                new Pose(91.000, 76.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(91.000, 76.000),
                                new Pose(111.000, 49.000),
                                new Pose(133.000, 60.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path7 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(133.000, 60.000),
                                new Pose(111.000, 49.000),
                                new Pose(91.000, 76.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        Path8 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(91.000, 76.000),
                                new Pose(93.000, 84.000),
                                new Pose(122.000, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-39), Math.toRadians(0))
                .build();

        Path9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(122.000, 83.000),
                                new Pose(91.000, 76.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path10 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(91.000, 76.000),
                                new Pose(111.000, 49.000),
                                new Pose(133.000, 60.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path11 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(133.000, 60.000),
                                new Pose(111.000, 49.000),
                                new Pose(91.000, 76.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ── INIT: begin driving, aim flywheel ──────────────────────────
            case 0:
                intake.setPower(0);
                kicker.moveToTarget();
                aimornot = true;
                follower.followPath(Path1);
                setPathState(1);
                break;

            // ── Wait for Path 1 to finish, then start intake ──────────────

            case 1:
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(3);
                }
                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path2, true);
                    setPathState(4);
                }
                break;

            // ── Wait for Path 2 to finish, then return to intake ──────────
            case 4:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    aimornot = true;
                    follower.followPath(Path3);
                    setPathState(5);
                }
                break;

            // ── Wait for Path 3 (return), start intake ────────────────────
            case 5:
                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(7);
                }
                break;

            // ── Intake dwell, then drive to shoot spot 2 (cycle 1 out) ───
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path4, true);
                    setPathState(8);
                }
                break;

            // ── Shoot spot 2 dwell (3s), then return to intake ────────────
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    intake.setPower(1);
                    aimornot = true;
                    follower.followPath(Path5);
                    setPathState(9);
                }
                break;

            // ── Wait for Path 5 (return), start intake ────────────────────
            case 9:
                if (!follower.isBusy()) {
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                        kicker.moveToHome();
                        intake.setPower(1);
                        setPathState(11);

                }
                break;
            // ── Shoot-2 cycle 2: intake dwell ─────────────────────────────
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path6, true);
                    setPathState(12);
                }
                break;

            // ── Shoot-2 cycle 2: dwell at spot, return ────────────────────
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    intake.setPower(1);
                    aimornot = true;
                    follower.followPath(Path7);
                    setPathState(13);
                }
                break;

            // ── Shoot-2 cycle 2 return: restart intake ────────────────────
            case 13:
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                        kicker.moveToHome();
                        intake.setPower(1);
                        setPathState(15);
                }
                break;
            // ── Shoot-2 cycle 3: intake dwell ─────────────────────────────
            case 15:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path8, true);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    aimornot = true;
                    follower.followPath(Path9);
                    setPathState(17);
                }
                break;

            // ── Shoot-2 cycle 3 return: restart intake ────────────────────
            case 17:
                if (!follower.isBusy()) {
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    intake.setPower(1);
                    kicker.moveToHome();
                    setPathState(19);
                }
                break;
            // ── Final intake dwell, then drive to endzone ─────────────────
            case 19:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path10);
                    setPathState(20);
                }
                break;

            // ── Arrived at endzone, aim and reverse to reset ──────────────
            case 20:
                if (pathTimer.getElapsedTimeSeconds() > 3) {
                    aimornot = true;
                    intake.setPower(0);
                    follower.followPath(Path11);
                    setPathState(21);
                }
                break;


            // ── Back at intake/reset position, start intake ───────────────
            case 21:
                if (!follower.isBusy()) {
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(23);
                }
                break;
            case 23:
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    kicker.moveToTarget();
                    intake.setPower(0);
                    aimornot = false;
                    setPathState(-1);
                }
                break;

            // ── Final shoot cycle: drive to shoot spot 2 and park ─────────


            default:
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public static double computeY(double x) {
        return ((0.000057964) * Math.pow(x, 4)
                - ((0.0197251)  * Math.pow(x, 3))
                + (2.38515 * Math.pow(x, 2))
                - 107.9964 * x
                + 3625.22859);
    }

    public static double computeHoodPosition(double x) {
        return (-((1.3307e-9)  * Math.pow(x, 4))
                + ((5.9712e-7) * Math.pow(x, 3))
                - (0.0000893847 * Math.pow(x, 2))
                + 0.00474244 * x
                + 0.250742);
    }
}