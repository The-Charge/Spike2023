// package frc.robot.subsystems;

// import org.photonvision.PhotonCamera;
// import org.photonvision.RobotPoseEstimator;
// import org.photonvision.RobotPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import java.util.ArrayList;
// import java.util.Collections;
// import java.util.List;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.FieldConstants;

// import edu.wpi.first.apriltag.AprilTag;
// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.math.Pair;
// import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

// import java.util.Optional;

// public class CamWrapper extends SubsystemBase {
//     private PhotonCamera camera;
//     List<CameraPoseObservation> observations;

//     public CamWrapper(PhotonCamera camera) {
//         this.camera = camera;
//         this.observations = new ArrayList<CameraPoseObservation>();
//     }

//     final Pose3d fieldPose = new Pose3d();


//     @Override
//     public void periodic() {
//       // This method will be called once per scheduler run
//       var res = camera.getLatestResult();
//       double observationTime = Timer.getFPGATimestamp() - res.getLatencyMillis();

//       List<PhotonTrackedTarget> tgtList = res.getTargets();

//       observations = new ArrayList<CameraPoseObservation>();

//       for(PhotonTrackedTarget t : tgtList){
//         Transform3d camToTargetTrans = t.getBestCameraToTarget(); //TODO - better apriltag multiple pose arbitration strategy
//         Transform3d targetTransform = Constants.VISION_NEAR_TGT_LOCATION; // TODO - needs to be looked up by AprilTag identifier
//         Pose3d targetPose = fieldPose.transformBy(targetTransform);
//         Pose3d camPose = targetPose.transformBy(camToTargetTrans.inverse());
//         Pose2d visionEstPose = camPose.transformBy(robotToCam.inverse()).toPose2d();   
//         observations.add(new CameraPoseObservation(observationTime, visionEstPose, 1.0)); //TODO - add trustworthiness scale by distance - further targets are less accurate  
//     }  



//     }
//     public class CameraPoseObservation {

//         public double time;//timer.getFPGATimestamp() referenced time when the observation was made
//         public Pose2d estFieldPose; //Estimated robot pose on the field at the time of the observation
//         public double trustworthiness; //Factor between 1.0 (fully trustworthy) and 0.0 (complete garbage) for this measurement. Abitrary units.
    
//         CameraPoseObservation(double time, Pose2d estFieldPose, double trustworthiness){
//             this.time = time;
//             this.estFieldPose =estFieldPose;
//             this.trustworthiness = trustworthiness;
//         }
        
//     }

// }

