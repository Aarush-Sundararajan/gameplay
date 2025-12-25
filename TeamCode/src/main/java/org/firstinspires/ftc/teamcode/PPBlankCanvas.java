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
public class PPBlankCanvas extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //Start Position_End Position
        //DRIVE -> MOVEMENT STATE
        //SHOOT -> ATTEMPT TO SCORE
        //PATHSTATE --> Names of positions you want your robot to move to ( states / positions )
        //Name them so it is clear what each position is
        DRIVE_STARTPOS_SHOOTPOS,

        DRIVE_SHOOTPOS_ENDPOS






    }

    PathState pathState;



    //Poses --> these are the specifics of the different positions
    //Poses are start positions for a path, end positions for a path, and key points you want to robot to pass over while driving in a curve
    private final Pose startPose = new Pose(56.4739336492891,102.71090047393365, Math.toRadians(90));

    private final Pose shootPose = new Pose(42.312796208530806,82.91943127962085, Math.toRadians(180));

    private final Pose endPose = new Pose(53.40284360189574,119.43127962085308, Math.toRadians(90));

    private final Pose endPoseC1 = new Pose(17.744075829383885,124.72037914691944,Math.toRadians(148));

    private final Pose endPoseC2 = new Pose(40.43601895734597,138.5402843601896, Math.toRadians(118));

    private PathChain driveStartPoseShootPose, driveShootPoseEndPos;






    public void buildPaths(){

        //put in coordinates for start pose --> end pose
        //2 types of move to points: BezierLine or BezierCurve
        //BezierLine --> simply 2 positions, start and end poses. Use this for linear movements that just require moving in a line ( angle is irrelevant )

        //BezierCurve  --> Uses a start and end pos with up to 3 middle poses that the robot must cover on its path. As the name implies, use this for curve movements
        //Try to name the intermediate poses C1, C2 etc...
        driveStartPoseShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();
        driveShootPoseEndPos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose,endPoseC1,endPoseC2,endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(),endPose.getHeading())
                .build();








    }

    //StatePathUpdate:
    //DO NOT CHANGE THE TWO OUTER BRACKETS
    //Too lazy to type here i will explain in the video
    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartPoseShootPose,true);
                setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                break;
            case DRIVE_SHOOTPOS_ENDPOS:
                    follower.followPath(driveShootPoseEndPos,true);
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
        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();;
        follower = Constants.createFollower(hardwareMap);
        //TODO add any other init ( Vihaan lock in )

        buildPaths();
        follower.setPose(startPose);

    }//set start cases and poses

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
