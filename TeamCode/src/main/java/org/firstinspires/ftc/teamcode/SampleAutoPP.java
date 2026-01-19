package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;
import com.pedropathing.geometry.BezierPoint;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.nio.file.Path;

//visualizer link: https://visualizer.pedropathing.com/

@TeleOp
public class SampleAutoPP extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //Start Position_End Position
        //DRIVE -> MOVEMENT STATE
        //SHOOT -> ATTEMPT TO SCORE
        //PATHSTATE --> Names of positions you want your robot to move to ( states / positions )
        //Name them so it is clear what each position is
        DRIVE_STARTPOS_COLLECT_POS,
        SHOOT_PRELOAD,
        SHOOT,
        DRIVE_COLLECTPOS_SHOOTPOS,

        DRIVE_SHOOTPOS_EndPOS



    }

    PathState pathState;



    //Poses --> these are the specifics of the different positions
    //Poses are start positions for a path, end positions for a path, and key points you want to robot to pass over while driving in a curve
    private final Pose startPose = new Pose(65.51658767772511,92.30331753554502,Math.toRadians(90));
    private final Pose collectPose = new Pose(35.14691943127962,84.11374407582937,Math.toRadians(180));

    private final Pose shootBPose = new Pose(60.90995260663507,104.24644549763033,Math.toRadians(136));

    private final Pose EndPose= new Pose(72,72.6824644549763,Math.toRadians(90));

    private final Pose EndPoseC1 = new Pose(91.7914691943128,110.9004739336493,Math.toRadians(121));

    private final Pose EndPoseC2 = new Pose(85.13744075829383,83.60189573459716,Math.toRadians(103));

    //PathChain --> Another list for the path positions ( different from PathState )
    private PathChain driveStartPosCollectPos, driveCollectPosShootPos, driveShootPosEndPose;

    public void buildPaths(){
        //put in coordinates for start pose --> end pose
        //2 types of move to points: BezierLine or BezierCurve
        //BezierLine --> simply 2 positions, start and end poses. Use this for linear movements that just require moving in a line ( angle is irrelevant )
        driveStartPosCollectPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,collectPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),collectPose.getHeading())
                .build();
        driveCollectPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(collectPose,shootBPose))
                .setLinearHeadingInterpolation(collectPose.getHeading(),shootBPose.getHeading())
                .build();
        //BezierCurve  --> Uses a start and end pos with up to 3 middle poses that the robot must cover on its path. As the name implies, use this for curve movements
        //Try to name the intermediate poses C1, C2 etc...
        driveShootPosEndPose = follower.pathBuilder()
                .addPath(new BezierCurve(shootBPose,EndPoseC1,EndPoseC2,EndPose))
                .setLinearHeadingInterpolation(shootBPose.getHeading(),EndPose.getHeading())
                .build();

    }

    //StatePathUpdate:
    //DO NOT CHANGE THE TWO OUTER BRACKETS
    //Too lazy to type here i will explain in the video
    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_COLLECT_POS:
                follower.followPath(driveStartPosCollectPos,true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                //check is follower done it's path
                //wait 5 sec
                if(!follower.isBusy()&& pathTimer.getElapsedTimeSeconds() > 5){
                    //TODO add logic to flywheel shooter ( Vihaan lock in )
                    follower.followPath(driveCollectPosShootPos);
                    setPathState(PathState.DRIVE_COLLECTPOS_SHOOTPOS);

                }
                break;
            case DRIVE_COLLECTPOS_SHOOTPOS:
                follower.followPath(driveCollectPosShootPos,true);
                setPathState(PathState.SHOOT);
                break;
            case SHOOT:
                //check is follower done it's path
                //wait 5 sec
                if(!follower.isBusy()&& pathTimer.getElapsedTimeSeconds() > 5){
                    //TODO add logic to flywheel shooter ( Vihaan lock in )
                    follower.followPath(driveShootPosEndPose);
                    setPathState(PathState.DRIVE_SHOOTPOS_EndPOS);

                }
                break;
            case DRIVE_SHOOTPOS_EndPOS:
                //all done!
                if(!follower.isBusy()){
                    telemetry.addLine("Done path 1");
                }
            default:
                telemetry.addLine("No state commanded");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }//keep same

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_COLLECT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();;
        follower = Constants.createFollower(hardwareMap);
        //TODO add any other init ( Vihaan lock in )

        buildPaths();
        follower.setPose(startPose);

    }//keep same

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }//if u change this u die

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addData("path state",pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading",follower.getPose().getHeading());
        telemetry.addData("Path Time",pathTimer.getElapsedTimeSeconds());
    }//telemetry is useless just keep everything else

}
