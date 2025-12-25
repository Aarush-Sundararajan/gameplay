package org.firstinspires.ftc.teamcode;



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



@TeleOp
public class PPCurveTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //Start Position_End Position
        //DRIVE -> MOVEMENT STATE
        //SHOOT -> ATTEMPT TO SCORE
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,




    }

    PathState pathState;


    private final Pose startPose = new Pose(43.84834123222749,125.40284360189573,Math.toRadians(138));
    private final Pose shootPose = new Pose(35.317535545023695,115.8483412322275,Math.toRadians(80));

    private final Pose c1 = new Pose(91.45023696682465,114.82464454976304,Math.toRadians(83));



    private PathChain driveStartPosShootPos;

    public void buildPaths(){
        //put in coordinates for start pose --> end pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,c1,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();

    }



    public void statePathUpdate(){
        switch(pathState){
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos,true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                //check is follower done it's path
                //wait 5 sec
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
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();;
        follower = Constants.createFollower(hardwareMap);
        //TODO add any other init ( Vihaan lock in )

        buildPaths();
        follower.setPose(startPose);

    }

    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addData("path state",pathState.toString());
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("heading",follower.getPose().getHeading());
        telemetry.addData("Path Time",pathTimer.getElapsedTimeSeconds());
    }

}
