package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.util.InterpLUT;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@Config
public class InternalPosition {
    private Supplier<Pose> robot;
    private DoubleSupplier turret;

    public static double turretOffset = 56.93625/25.4, cameraOffset = 180.97618/25.4;
    public static Pose blueGoal = new Pose(12,134), redGoal = mirrorIf(blueGoal, true);
    public InternalPosition(Supplier<Pose> robotPosition, DoubleSupplier turretRotation) {
        robot = robotPosition;
        turret = turretRotation;
    }

    public double getDistance() {
        return robot.get().distanceFrom(goal());
    }

    public double getTurretAngle() {
        Pose r = robot.get();
        Pose g = goal();
        return Math.atan2(g.getY() - r.getY(), g.getX() - r.getX());
    }

    public Pose goal() {
        switch (StateTransfer.alliance) {
            case Red: return redGoal;
            case Blue: return blueGoal;
        }
        return null;
    }

    public static Pose mirrorIf(Pose oldPose, boolean flip) {
        if (flip) {
            return oldPose.mirror().withX(141.5 - oldPose.getX());
        } else {
            return oldPose;
        }
    }

    public static Pose mirrorIf(double x, double y, double degrees, boolean flip) {
        return mirrorIf(new Pose(x, y, Math.toRadians(degrees)), flip);
    }

    public static double mirrorAngleIf(double theta, boolean flip) {
        if (flip) {
            return MathFunctions.normalizeAngle(Math.PI - theta);
        } else {
            return theta;
        }
    }
}
