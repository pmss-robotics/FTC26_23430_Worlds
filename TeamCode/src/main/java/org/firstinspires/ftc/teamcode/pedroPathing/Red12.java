package org.firstinspires.ftc.teamcode.pedroPathing; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous(name = "RED |12|", group = "Autonomous")
public class Red12 extends OpMode {
    KickerSubsystem kicker;
    private Servo indicator;
    IntakeSubsystemNew intake;
    HoodSubsystem hood;
    FlywheelSubsystem outtake;
    TurretSubsystem turret;
    private static boolean aimornot = false;
    public static double targetX = 143;
    public static double targetY = 143;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose pose1 = new Pose(115.956, 130.013, Math.toRadians(-142));
    private final Pose pose2 = new Pose(97, 84, Math.toRadians(0));
    private final Pose pose3 = new Pose(126, 84, Math.toRadians(0));
    private final Pose pose4 = new Pose(85, 70, Math.toRadians(0));
    private final Pose pose5 = new Pose(100, 60, Math.toRadians(0));
    private final Pose pose6 = new Pose(124, 60, Math.toRadians(0));
    private final Pose pose7 = new Pose(85, 70, Math.toRadians(0));
    private final Pose pose8 = new Pose(85, 70, Math.toRadians(0));
    private final Pose pose9 = new Pose(132, 36, Math.toRadians(0));
    private final Pose pose10 = new Pose(85, 70, Math.toRadians(0));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2Start, grabPickup2End, scorePickup2, grabPickup3Start, grabPickup3End, scorePickup3;

    @Override
    public void init() {
        kicker = new KickerSubsystem(hardwareMap);
        intake = new IntakeSubsystemNew(hardwareMap, telemetry);
        hood = new HoodSubsystem(hardwareMap, telemetry);
        outtake = new FlywheelSubsystem(hardwareMap, telemetry);
        turret = new TurretSubsystem(hardwareMap);
        indicator = hardwareMap.get(Servo.class, "indicator");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(pose1);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop()  {
        follower.update();
        autonomousPathUpdate();




        double distance = turret.getDistance();
        double hoodPos = computeHoodPosition(distance);

        hoodPos = Range.clip(hoodPos, 0.0, 1.0);
        hood.setHoodPosition(hoodPos);

        if (aimornot) {
            outtake.setVelocityRpm(computeY(turret.getDistance()));
        } else {
            outtake.setVelocityRpm(0);
        }

        Pose pose = follower.getPose();
        double robotX = pose.getX();
        double robotY = pose.getY();
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        double aimAngle = turret.calculateAimAngle(
                robotX, robotY, robotHeadingDeg,
                targetX, targetY
        );

        turret.goToAngle(aimAngle);

        double turretAngle = turret.getTurretAngle();
        double error = Math.abs(aimAngle - turretAngle);

        if (error < 2) {
            indicator.setPosition(0.611);
        } else {
            indicator.setPosition(0.277);
        }

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Turret Angle", turretAngle);
        telemetry.addData("Aim Angle", aimAngle);
        telemetry.addData("Error", error);
        telemetry.addData("Flywheel RPM", outtake.getTargetRPM());
        telemetry.addData("Actual Outtake RPM", outtake.getActualRpm());
        telemetry.addData("Actual Outtake 2 RPM", outtake.getActualRpm2());
        telemetry.addData("Distance to goal", turret.getDistance());
        telemetry.update();
    }

    @Override
    public void stop() {
        // No special stop actions needed
    }

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(pose1, pose2));
        scorePreload.setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, pose4))
                .setLinearHeadingInterpolation(pose3.getHeading(), pose4.getHeading())
                .build();

        grabPickup2Start = follower.pathBuilder()
                .addPath(new BezierLine(pose4, pose5))
                .setLinearHeadingInterpolation(pose4.getHeading(), pose5.getHeading())
                .build();

        grabPickup2End = follower.pathBuilder()
                .addPath(new BezierLine(pose5, pose6))
                .setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pose6, pose7))
                .setLinearHeadingInterpolation(pose6.getHeading(), pose7.getHeading())
                .build();

        grabPickup3Start = follower.pathBuilder()
                .addPath(new BezierLine(pose7, pose8))
                .setLinearHeadingInterpolation(pose7.getHeading(), pose8.getHeading())
                .build();

        grabPickup3End = follower.pathBuilder()
                .addPath(new BezierLine(pose8, pose9))
                .setLinearHeadingInterpolation(pose8.getHeading(), pose9.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pose9, pose10))
                .setLinearHeadingInterpolation(pose9.getHeading(), pose10.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                kicker.moveToTarget();
                aimornot = true;
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(grabPickup1, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    aimornot = true;
                    follower.followPath(scorePickup1);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    intake.setPower(0);
                    follower.followPath(grabPickup2Start);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(grabPickup2End, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    aimornot = true;
                    intake.setPower(0);
                    follower.followPath(scorePickup2);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    intake.setPower(0);
                    follower.followPath(grabPickup3Start);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    intake.setPower(1);
                    follower.followPath(grabPickup3End, true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    aimornot = true;
                    intake.setPower(0);
                    follower.followPath(scorePickup3);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    intake.setPower(0);
                    setPathState(-1);
                }
                break;
            default:
                //Do nothing or end autonomous
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public static double computeY(double x) {
        return (-0.0000099416 * Math.pow(x, 4)) + (0.00464449 * Math.pow(x, 3)) - (0.79584 * Math.pow(x, 2)) + (73.70122 * x) - 132.82571;
    }

    // ⭐ HOOD QUADRATIC FORMULA
    public static double computeHoodPosition(double x) {
        return 1.04895 * Math.pow(10, -8) * Math.pow(x, 4) - 0.00000362859 * Math.pow(x, 3) + 0.000446154 * Math.pow(x, 2) - 0.0232164 * x + 0.77352;
    }

}