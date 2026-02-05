package org.firstinspires.ftc.teamcode; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Autonomous
public class EmptyAuto extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose2 = new Pose(56.948929159802304, 104.522, Math.toRadians(180)); // Start Pose of our robot.

    private final Pose startPose = new Pose(32.836, 134.522, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(48.273476112026366, 85.44645799011532, Math.toRadians(180)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePose2 = new Pose(55.00329489291597, 81.95238879736408, Math.toRadians(230)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.5
    private final Pose pickup1Pose = new Pose(20.13179571663921, 81.44645799011532, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(16.856672158154865, 60.1166392092257, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(10.865671641791039, 62.3597014925373, Math.toRadians(155)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    private final Pose pickup2restPose = new Pose(43.582089552238806, 60.1166392092257, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, scorePickup4;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose2));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose2.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//                .setConstraints(Constants.slowPathConstraints)
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, startPose2))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), startPose2.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup2restPose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup2restPose.getHeading())
//                .setConstraints(Constants.slowPathConstraints)
                .addPath(new BezierLine(pickup2restPose, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2restPose.getHeading(), pickup2Pose.getHeading())
//                .setConstraints(Constants.slowPathConstraints)
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, pickup2restPose, scorePose2))
                .setLinearHeadingInterpolation(pickup2restPose.getHeading(), scorePose2.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose2.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, pickup2restPose, scorePose2))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose2.getHeading())
                .build();
        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, pickup2restPose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to scoring position

                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1: // Wait for arrival at scoring position
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 2.25) {
                    setPathState(2);
                }
                break;
            case 2: // Shoot preloads (5 seconds as requested)

                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    setPathState(10);
                }
                break;
            case 3: // Move to pickup position
                follower.followPath(grabPickup1, true);

                setPathState(4);
                break;
            case 4: // Wait for arrival at pickup position
                if(!follower.isBusy()) {
                    setPathState(5);
                }
                break;
            case 5: // Intake for 1 second (as requested)
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(6);
                }
                break;
            case 6: // Stop intake, spin up shooter, move to final score position
                follower.followPath(scorePickup1, true);
                setPathState(7);
                break;
            case 7: // Wait for arrival at final score position

                if(!follower.isBusy()) {

                    setPathState(8);
                }
                break;
            case 8: // Wait for shooter spin-up post-movement
                if(pathTimer.getElapsedTimeSeconds() > 0.75) { // 1 second delay for spin-up
                    setPathState(9);
                }
                break;
            case 9: // Final shooting

                if(pathTimer.getElapsedTimeSeconds() > 1) { // Allow time for final shots
                    setPathState(-1);
                }
                break;
            case 10: // Move to pickup position
                follower.followPath(grabPickup2, 0.75, true);

                setPathState(11);
                break;
            case 11: // Wait for arrival at pickup position
                if(!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case 12: // Intake for 1 second (as requested)
                if(pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(13);
                }
                break;
            case 13: // Stop intake, spin up shooter, move to final score position

                follower.followPath(scorePickup2, true);
                setPathState(14);

                break;
            case 14: // Wait for arrival at final score position

                if(!follower.isBusy()) {

                    setPathState(15);
                }
                break;
            case 15: // Wait for shooter spin-up post-movement
                if(pathTimer.getElapsedTimeSeconds()>0.5) { // 1 second delay for spin-up
                    setPathState(16);
                }
                break;
            case 16: // Final shooting

                if(pathTimer.getElapsedTimeSeconds() > 1) { // Allow time for final shots
                    setPathState(17);
                }
                break;

            case 17: // Move to pickup position
                follower.followPath(grabPickup3, true);

                setPathState(18);
                break;
            case 18: // Wait for arrival at pickup position
                if(!follower.isBusy()) {
                    setPathState(19);
                }
                break;
            case 19: // Intake for 1 second (as requested)
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(21);
                }
                break;
            case 21: // Stop intake, spin up shooter, move to final score position
                follower.followPath(scorePickup3, true);
                setPathState(22);

                break;
            case 22: // Wait for arrival at final score position

                if(!follower.isBusy()) {

                    setPathState(23);
                }
                break;
            case 23: // Wait for shooter spin-up post-movement
                if(pathTimer.getElapsedTimeSeconds()>0.7) { // 1 second delay for spin-up
                    setPathState(24);
                }
                break;
            case 24: // Final shooting

                if(pathTimer.getElapsedTimeSeconds() > 1) { // Allow time for final shots
                    setPathState(25);
                }
                break;
            case 25: // Move to pickup position
                follower.followPath(grabPickup3, true);

                setPathState(26);
                break;
            case 26: // Wait for arrival at pickup position
                if(!follower.isBusy()) {
                    setPathState(27);
                }
                break;
            case 27: // Intake for 1 second (as requested)
                if(pathTimer.getElapsedTimeSeconds() > 1.5) {
                    setPathState(28);
                }
                break;
            case 28: // Stop intake, spin up shooter, move to final score position
                follower.followPath(scorePickup3, true);
                setPathState(29);

                break;
            case 29: // Wait for arrival at final score position

                if(!follower.isBusy()) {

                    setPathState(31);
                }
                break;
            case 31: // Wait for shooter spin-up post-movement
                if(pathTimer.getElapsedTimeSeconds()>0.7) { // 1 second delay for spin-up
                    setPathState(32);
                }
                break;
            case 32: // Final shooting

                if(pathTimer.getElapsedTimeSeconds() > 1) { // Allow time for final shots
                    setPathState(3);
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // Update mechanisms
        follower.update();
//        turret.update(follower.getPose());
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("target RPM", "Dynamic");
//        telemetry.addData("vel", shooter.getvel());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        buildPaths();
        follower.setStartingPose(startPose);


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}