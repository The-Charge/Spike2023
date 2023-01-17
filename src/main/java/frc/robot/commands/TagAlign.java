package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.RobotPoseEstimator;
public class TagAlign extends CommandBase{
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
    
    private final PhotonCamera photonCamera;
    private final Drivetrain drivetrain;
    private static final Transform3d TAG_TO_GOAL = new Transform3d(new Translation3d(1.5, 0.0, 0.0), new Rotation3d(0.0, 0.0, Math.PI));
    private final PoseEstimator poseProvider; 
    //Supplier<Pose2d> poseProvider
    private double TAG_TO_CHASE = 4; //Change depending on what this command will be used for. Probably for substation tags (4/5)
    private PhotonTrackedTarget lastTarget;

    //Controllers for x/y/rotational movement
    private final ProfiledPIDController xController = new ProfiledPIDController(0.8, 0, 0.001, null);
    private final ProfiledPIDController yController = new ProfiledPIDController(0.8, 0, 0.001, null);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);
    
    public TagAlign(PhotonCamera photonCamera, Drivetrain drivetrain, PoseEstimator poseProvider) {
        this.photonCamera = photonCamera;
        this.drivetrain = drivetrain;
        this.poseProvider = poseProvider;

        xController.setTolerance(0.2);
        yController.setTolerance(0.2);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain);
    }
    @Override
    public void initialize() {
        //reset current pose
        lastTarget = null;
        var robotPose = poseProvider.getCurrentPose();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }
    @Override
    public void execute() {
        //get current poses
        var robotPose2d = poseProvider.getCurrentPose();
        var robotPose = new Pose3d(robotPose2d.getX(), robotPose2d.getY(), 0.0, new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
        //new camera input
        var photonResult = photonCamera.getLatestResult();
        PhotonTrackedTarget targetOpt = null;
        if (photonResult.hasTargets()) {
            targetOpt = photonResult.getBestTarget();
            //var targetOpt = photonResult.getTargets().stream().filter(t -> t.getFiducialId() == TAG_TO_CHASE).filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= 0.2 && t.getPose);
        }
    if (targetOpt != null && targetOpt.getFiducialId() == TAG_TO_CHASE) {
        var target = targetOpt;
        lastTarget = target;

        //Pose transformations robotPose -> cameraPose -> apriltagPose -> goalPose
        var cameraPose = robotPose.transformBy(VisionConstants.robotToCam); //transform by height of camera on robo
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);
        var goalPose = targetPose.transformBy(TAG_TO_GOAL);

        //Set controller goal destinations
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(Units.degreesToRadians(goalPose.getRotation().getAngle()));


        }if (lastTarget == null) {
            drivetrain.stop();
        }
    else {
        var xSpeed = xController.calculate(robotPose.getX());
        if (xController.atGoal()) {
            xSpeed = 0;
        }
        var ySpeed = yController.calculate(robotPose.getY());
        if (yController.atGoal()) {
            ySpeed = 0;
        }
        var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }
        drivetrain.runVelocityMode(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
    }
    }
}