package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.FieldConstants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

import java.util.Optional;
import frc.robot.Constants.DriveConstants;

public class PoseEstimator extends SubsystemBase {
    private final PhotonCamera photonCamera;
    private final Drivetrain drivetrain;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private double previousPipelineTimeStamp = 0;

    //Target(AprilTag) Poses need to be changed based on positions of the Tags on the Playing Field
    private static final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of(new Pose3d(3.0,1.165,0.287+0.165,new Rotation3d(0,0, Units.degreesToRadians(180)))));

    // private static final Vector<N5>  stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(0));
    // private static final Vector<N5> localMeasurementStdDevs = VecBuilder.fill();

    public PoseEstimator(PhotonCamera photonCamera, Drivetrain drivetrain) {
        this.photonCamera = photonCamera;
        this.drivetrain = drivetrain;
        //To-do: Add stdDevs for trust values
        poseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(Units.inchesToMeters(DriveConstants.kTrackwidthMeters)), this.drivetrain.getGyroRotation2d(), this.drivetrain.getLeftEncoder(), this.drivetrain.getRightEncoder(), getCurrentPose());
    }
    
    @Override
    public void periodic() {
        var pipelineResult = photonCamera.getLatestResult();
        var resultTimestamp = pipelineResult.getTimestampSeconds();
        //check if it's been a new time + camera has found results
        if (resultTimestamp != previousPipelineTimeStamp && pipelineResult.hasTargets()) {
            photonCamera.setLED(VisionLEDMode.kBlink);
            previousPipelineTimeStamp = resultTimestamp;
            //swap to the "best" target from PhotonLib
            var target = pipelineResult.getBestTarget();
            var fiducialId = target.getFiducialId();
            SmartDashboard.putNumber("FiducialId", fiducialId);
            if (target.getPoseAmbiguity() <= 0.2 && fiducialId >= 0){ //&& fiducialId < targets.size()) <- figure out region where tags are located. 
                //apriltag targets can probably be placed as part of the constants
                var targetPose = targetPoses.get(fiducialId);
                Transform3d camToTarget = target.getBestCameraToTarget();
                Pose3d camPose = targetPose.transformBy(camToTarget.inverse());

                var VisionMeasurement = camPose.transformBy(VisionConstants.camToRobot); //transform by height of camera on the bot relative to the floor from Cam down to Bot
                poseEstimator.addVisionMeasurement(VisionMeasurement.toPose2d(), resultTimestamp); 
                SmartDashboard.putString("Final Transform", VisionMeasurement.toString());
            }
        }
        else {
            photonCamera.setLED(VisionLEDMode.kOn);
            SmartDashboard.putNumber("FiducialId", -1); //test
        }
        SmartDashboard.putString("Current Pose", poseEstimator.getEstimatedPosition().toString());
    }
    public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
    }
}