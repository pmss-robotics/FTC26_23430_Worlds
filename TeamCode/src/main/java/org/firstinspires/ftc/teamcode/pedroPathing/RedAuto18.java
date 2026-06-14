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

@Autonomous(name = "|R|18|", group = "Autonomous")
public class RedAuto18 extends OpMode {
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
    // Path2-Path7, Path10: the main route segments
    private PathChain Path2, Path3, Path4, Path5, Path6, Path7, Path10;

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

        follower.setStartingPose(new Pose(125.000, 110.000, Math.toRadians(0)));
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
                new Pose(125.000, 110.000),
                new Pose(113.000, 103.000),
                new Pose(95.000, 83.000)));
        Path1.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-30));

        // Path 2: Intake zone (95,83) → first shoot spot (118,59)
        // Curved — heading from -30° to 0°
        Path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(95.000, 83.000),
                        new Pose(99.000, 56.000),
                        new Pose(126.000, 59.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(0))
                .build();

        // Path 3: First shoot spot (118,59) → intake zone (94,77)
        // Straight return
        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(126.000, 59.000),
                        new Pose(83.000, 69.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 4: Intake zone (94,77) → second shoot spot (129,59)
        // Straight drive with heading sweep to 25°
        Path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(83.000, 69.000),
                        new Pose(131.800, 58.2)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                .build();

        // Path 5: Second shoot spot (129,59) → intake zone (94,77)
        // Straight return, heading back to 0°
        Path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(131.800, 58.2),
                        new Pose(83.000, 69.000)))
                .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))
                .build();

        // Path 6: Intake zone (94,77) → endzone collect spot (118,82)
        // Curved sweep along the wall
        Path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(83.000, 69.000),
                        new Pose(90.000, 82.000),
                        new Pose(122.000, 82.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 7: Endzone spot (118,82) → reset/intake (94,77)
        // Straight reverse
        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(122.000, 82.000),
                        new Pose(100.000, 82.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Path 10: Final shoot run (94,77) → second shoot spot (129,59), then park
        // Reuses the shoot-2 angle for one last cycle before time runs out
        Path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(94.000, 77.000),
                        new Pose(129.000, 59.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))
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
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(2);
                }
                break;

            // ── Intake dwell (1.2s), then drive to shoot spot 1 ───────────
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path2, true);
                    setPathState(3);
                }
                break;

            // ── Wait for Path 2 to finish, then return to intake ──────────
            case 3:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    aimornot = true;
                    follower.followPath(Path3);
                    setPathState(4);
                }
                break;

            // ── Wait for Path 3 (return), start intake ────────────────────
            case 4:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(5);
                }
                break;

            // ── Intake dwell, then drive to shoot spot 2 (cycle 1 out) ───
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path4, true);
                    setPathState(6);
                }
                break;

            // ── Shoot spot 2 dwell (3s), then return to intake ────────────
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 3.3) {
                    intake.setPower(1);
                    aimornot = true;
                    follower.followPath(Path5);
                    setPathState(7);
                }
                break;

            // ── Wait for Path 5 (return), start intake ────────────────────
            case 7:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(1005);
                }
                break;

            // ── Shoot-2 cycle 2: intake dwell ─────────────────────────────
            case 1005:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path4, true);
                    setPathState(1006);
                }
                break;

            // ── Shoot-2 cycle 2: dwell at spot, return ────────────────────
            case 1006:
                if (pathTimer.getElapsedTimeSeconds() > 3.3) {
                    intake.setPower(1);
                    aimornot = true;
                    follower.followPath(Path5);
                    setPathState(1007);
                }
                break;

            // ── Shoot-2 cycle 2 return: restart intake ────────────────────
            case 1007:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(2005);
                }
                break;

            // ── Shoot-2 cycle 3: intake dwell ─────────────────────────────
            case 2005:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path4, true);
                    setPathState(2006);
                }
                break;

            // ── Shoot-2 cycle 3: slightly shorter dwell (2.8s), return ───
            case 2006:
                if (pathTimer.getElapsedTimeSeconds() > 3.3) {
                    intake.setPower(1);
                    aimornot = true;
                    follower.followPath(Path5);
                    setPathState(2007);
                }
                break;

            // ── Shoot-2 cycle 3 return: restart intake ────────────────────
            case 2007:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(8);
                }
                break;

            // ── Final intake dwell, then drive to endzone ─────────────────
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path6);
                    setPathState(9);
                }
                break;

            // ── Arrived at endzone, aim and reverse to reset ──────────────
            case 9:
                if (!follower.isBusy()) {
                    aimornot = true;
                    intake.setPower(0);
                    follower.followPath(Path7);
                    setPathState(10);
                }
                break;

            // ── Back at intake/reset position, start intake ───────────────
            case 10:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
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