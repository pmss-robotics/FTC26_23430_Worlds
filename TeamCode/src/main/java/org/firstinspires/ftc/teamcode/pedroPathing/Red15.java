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
import com.seattlesolvers.solverslib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.*;

@Autonomous(name = "RED |15|", group = "Autonomous")
public class Red15 extends OpMode {
    KickerSubsystem kicker;
    private Servo indicator;
    IntakeSubsystemNew intake;
    HoodSubsystem hood;
    FlywheelSubsystem outtake;
    TurretSubsystem turret;
    private  boolean aimornot = false;
    public static double targetX = 143;
    public static double targetY = 143;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;



    private Path Path1;
    private PathChain Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

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
        follower.setStartingPose(new Pose(115.956, 130.013, Math.toRadians(34)));
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
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        double distance = turret.getDistance();
        double hoodPos = computeHoodPosition(distance);

        hoodPos = Math.max(0.0, Math.min(1.0, hoodPos));
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
    }

    @Override
    public void stop() {
        // No special stop actions needed
    }
    public void  buildPaths() {
                Path1 = new Path(new BezierCurve(
                        new Pose(115.956, 130.013),
                        new Pose(89.000, 97.000),
                        new Pose(84.000, 67.000)
                ));
                Path1.setLinearHeadingInterpolation(Math.toRadians(34), Math.toRadians(0));


                Path2 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(84.000, 67.000),
                                        new Pose(110.000, 58.000),
                                        new Pose(120.000, 59.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path3 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(120.000, 59.000),
                                        new Pose(110.000, 58.000),
                                        new Pose(84.000, 67.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path4 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(84.000, 67.000),

                                        new Pose(130.000, 58.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(28))

                        .build();

                Path5 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(130.000, 58.000),
                                        new Pose(97.000, 63.000),
                                        new Pose(100.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(28), Math.toRadians(0))

                        .build();

                Path6 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(100.000, 84.000),

                                        new Pose(120.000, 84.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path7 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(120.000, 84.000),

                                        new Pose(84.000, 67.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path8 = follower.pathBuilder().addPath(
                                new BezierCurve(
                                        new Pose(84.000, 67.000),
                                        new Pose(97.000, 31.000),
                                        new Pose(120.000, 35.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

                Path9 = follower.pathBuilder().addPath(
                                new BezierLine(
                                        new Pose(120.000, 35.000),

                                        new Pose(84.000, 67.000)
                                )
                        ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                        .build();

        }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                intake.setPower(0);
                kicker.moveToTarget();
                aimornot = true;
                follower.followPath(Path1);
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
                    follower.followPath(Path2, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    aimornot = true;
                    follower.followPath(Path3);
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
                    follower.followPath(Path4);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    aimornot = true;
                    follower.followPath(Path5);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path6);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    aimornot = true;
                    intake.setPower(0);
                    follower.followPath(Path7);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    follower.followPath(Path8);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    aimornot = true;
                    intake.setPower(0);
                    follower.followPath(Path9);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    kicker.moveToHome();
                    intake.setPower(1);
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    kicker.moveToTarget();
                    aimornot = false;
                    intake.setPower(0);
                    follower.followPath(Path8);
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