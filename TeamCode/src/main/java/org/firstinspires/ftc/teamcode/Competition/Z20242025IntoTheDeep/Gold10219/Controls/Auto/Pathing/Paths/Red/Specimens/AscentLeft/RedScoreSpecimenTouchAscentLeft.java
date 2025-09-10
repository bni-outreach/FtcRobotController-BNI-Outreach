package org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft;

import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.ascentLExtendArm;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.ascentLExtendArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.ascentLGrabberOut;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.ascentLLowerArm;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.ascentLTimeout;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.chambers1Back;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.chambers1BackTimeout;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.chambers1Heading;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.chambers1LowerArm;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.chambers1LowerArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.chambers1RaiseArm;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.chambers1RaiseArmTimeout;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.chambers1Release;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.chambers1Timeout;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.holdAscentL;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.holdChambers1;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.opModeStop;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.toAscentL;
import static org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Paths.Red.Specimens.AscentLeft.PathStates.toChambers1;

//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.BotPose.Pinpoint;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.BotPose.PoseHelper;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.BotPose.Vision;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Controls.Auto.Pathing.Vars.FieldPoses;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Mechanisms.Grabber.Grabber;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Mechanisms.PrimaryArm.PrimaryArm;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Robots.CompBot.CompBot;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Robots.CompBot.CompBotVars;
//import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Utils.EasyPoint;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Utils.O;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Utils.Offsets;
//import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Utils.Paths.EasySafePath;
import org.firstinspires.ftc.teamcode.Competition.Z20242025IntoTheDeep.Gold10219.Utils.Paths.HeadingTypes;

import java.util.HashMap;
import java.util.Map;

//@Autonomous(name = "Red Score Specimen Touch Ascent Left", group = "Auto - Red - Specimen")
//public class RedScoreSpecimenTouchAscentLeft extends OpMode {
//    private final CompBot Bot = new CompBot();
//    private final CompBotVars vars = new CompBotVars();
//
//    private final Vision vision = new Vision();
//    private final Pinpoint pinpoint = new Pinpoint();
//    private final PoseHelper pose = new PoseHelper();
//
//    private final FieldPoses poses = new FieldPoses();
//
//    Grabber grabber = new Grabber();
//
//    PrimaryArm arm = new PrimaryArm();
//
//    private Pose startPose;
//
//    private Follower follower;
//
//    private Timer pathTimer;
//
//    private final Map<PathStates, Path> paths = new HashMap<>();
//    private PathStates pathState;
//
//    @Override
//    public void init() {
//        Bot.initRobot(hardwareMap);
//
//        pinpoint.setOp(this);
//        pinpoint.initPinpoint(hardwareMap);
//
//        vision.setOp(this);
//        vision.initVision(hardwareMap, pinpoint);
//
//        pose.setOp(this);
//        pose.setDevices(vision, pinpoint);
//
//        pathTimer = new Timer();
//
//        vision.start();
//
//        pose.updateLLUsage(false);
//
//        Pose2D currentPose = pose.getSmartPose(PoseHelper.Alliances.RED);
//        telemetry.addData("Pose X: ", currentPose.getX(DistanceUnit.INCH));
//        telemetry.addData("Pose Y: ", currentPose.getY(DistanceUnit.INCH));
//        telemetry.addData("Pose H: ", currentPose.getHeading(AngleUnit.DEGREES));
//
//        startPose = new Pose(
//                currentPose.getX(DistanceUnit.INCH),
//                currentPose.getY(DistanceUnit.INCH),
//                currentPose.getHeading(AngleUnit.RADIANS)
//        );
//
//        telemetry.addData("Start Pose: ", startPose);
//        telemetry.update();
//
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(startPose);
//
//        arm.initPrimaryArm(hardwareMap, Bot.LinearOp);
//        grabber.initGrabber(hardwareMap);
//
//        grabber.grab();
//        grabber.headStraight();
//        grabber.setGrabberState(Grabber.grabberStates.TUCK);
//        arm.setRetract();
//    }
//
//    public void start() {
//        grabber.grab();
//        grabber.headStraight();
//        grabber.setGrabberState(Grabber.grabberStates.TUCK);
//        arm.setRetract();
//        buildPaths();
//        setPathState(chambers1RaiseArm);
//    }
//
//    public void loop() {
//        pose.updatePose();
//        follower.update();
//        grabber.tiltStateCheck();
//        arm.rotationChecker();
//        autonomousPathUpdate();
//        tel();
//    }
//
//    Pose l = null;
//
//    public void tel() {
//        Pose2D current = pose.getPose();
//        telemetry.addData("PX: ", current.getX(DistanceUnit.INCH));
//        telemetry.addData("PY: ", current.getY(DistanceUnit.INCH));
//        telemetry.addData("PO: ", current.getHeading(AngleUnit.DEGREES));
//        telemetry.addLine();
//        telemetry.addData("pathState: ", pathState);
//        if (l != null) {
//            telemetry.addLine();
//            telemetry.addData("LX: ", l.getX());
//            telemetry.addData("LY: ", l.getY());
//            telemetry.addData("LH: ", l.getHeading());
//        }
//        telemetry.update();
//    }
//
//    private Path getPath(PathStates state) {
//        return O.req(paths.get(state));
//    }
//
//    public void buildPaths() {
//        paths.put(toChambers1,
//                new EasySafePath(startPose, poses.Chambers.Red,
//                        new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.AtChambers.OUT))
//                        .setHeading(HeadingTypes.CONSTANT, startPose));
//
//        paths.put(toAscentL,
//                new EasySafePath(getPath(toChambers1).getLastControlPoint(), poses.Ascents.Pre.Red, poses.Ascents.Post.Red, poses.Ascents.Red.Left,
//                        new Offsets().remX(vars.Chassis.FRONT_LENGTH))
//                        .setHeading(HeadingTypes.LINEAR, getPath(toChambers1), poses.Ascents.Red.Left, .35));
//    }
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case chambers1RaiseArm:
//                arm.setRotation(PrimaryArm.rotationStates.UP, 6, true);
//                grabber.setGrabberState(Grabber.grabberStates.OUT);
//                setPathState(chambers1RaiseArmTimeout);
//                break;
//            case chambers1RaiseArmTimeout:
//                if (pathTimer.getElapsedTime() > 1000) {
//                    setPathState(toChambers1);
//                }
//                break;
//            case toChambers1:
//                follower.followPath(getPath(toChambers1));
//                setPathState(chambers1Heading);
//                break;
//            case chambers1Heading:
//                if (follower.getCurrentTValue() > 0.1) {
//                    ((EasySafePath) getPath(toChambers1)).setHeading(HeadingTypes.LINEAR, startPose, poses.Chambers.Red);
//                    setPathState(holdChambers1);
//                }
//                break;
//            case holdChambers1:
//                if (!follower.isBusy()) {
//                    follower.holdPoint(
//                            new EasyPoint(poses.Chambers.Red,
//                                    new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.AtChambers.OUT)),
//                            poses.Chambers.Red.getHeading()
//                    );
//                    setPathState(chambers1Timeout);
//                }
//                break;
//            case chambers1Timeout:
//                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
//                    setPathState(chambers1LowerArm);
//                }
//                break;
//            case chambers1LowerArm:
//                grabber.setGrabberState(Grabber.grabberStates.HOOK);
//                if (pathTimer.getElapsedTime() > 500) {
//                    arm.setRotation(PrimaryArm.rotationStates.DOWN, .75, false);
//                    setPathState(chambers1LowerArmTimeout);
//                }
//                break;
//            case chambers1LowerArmTimeout:
//                if (pathTimer.getElapsedTime() > 500) {
//                    setPathState(chambers1Release);
//                }
//                break;
//            case chambers1Release:
//                grabber.release();
//                grabber.setGrabberState(Grabber.grabberStates.OUT);
//                arm.up(.5, false);
//                setPathState(chambers1Back);
//                break;
//            case chambers1Back:
//                grabber.release();
//                grabber.setGrabberState(Grabber.grabberStates.OUT);
//                follower.holdPoint(
//                        new EasyPoint(poses.Chambers.Retreats.Red,
//                                new Offsets().remY(vars.Chassis.FRONT_LENGTH).remY(vars.Mechanisms.Grabber.AtChambers.OUT)),
//                        poses.Chambers.Retreats.Red.getHeading()
//                );
//                setPathState(chambers1BackTimeout);
//            case chambers1BackTimeout:
//                if (pathTimer.getElapsedTime() > 500) {
//                    setPathState(toAscentL);
//                }
//            case toAscentL:
//                follower.followPath(getPath(toAscentL));
//                setPathState(holdAscentL);
//                break;
//            case holdAscentL:
//                if (!follower.isBusy()) {
//                    follower.holdPoint(
//                            new EasyPoint(poses.Ascents.Red.Left,
//                                    new Offsets().remX(vars.Chassis.FRONT_LENGTH)),
//                            poses.Ascents.Red.Left.getHeading()
//                    );
//                    setPathState(ascentLGrabberOut);
//                }
//                break;
//            case ascentLGrabberOut:
//                grabber.setGrabberState(Grabber.grabberStates.OUT);
//                setPathState(ascentLTimeout);
//                break;
//            case ascentLTimeout:
//                if (pathTimer.getElapsedTime() > 500 && !follower.isBusy()) {
//                    setPathState(ascentLExtendArm);
//                }
//                break;
//            case ascentLExtendArm:
//                arm.setExtend();
//                setPathState(ascentLExtendArmTimeout);
//                break;
//            case ascentLExtendArmTimeout:
//                if (pathTimer.getElapsedTime() > 500) {
//                    setPathState(ascentLLowerArm);
//                }
//                break;
//            case ascentLLowerArm:
//                arm.setRotation(PrimaryArm.rotationStates.DOWN, 2.5, false);
//                setPathState(opModeStop);
//                break;
//            case opModeStop:
//                if (arm.getRotationState() == PrimaryArm.rotationStates.STOPPED) {
//                    requestOpModeStop();
//                }
//                break;
//        }
//    }
//
//    public void setPathState(PathStates state) {
//        pathState = state;
//        pathTimer.resetTimer();
//        autonomousPathUpdate();
//    }
//}
