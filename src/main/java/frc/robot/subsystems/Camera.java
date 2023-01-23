package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

//Camera subsystem to encompass both PhotonCameras
public class Camera extends SubsystemBase{
    private PhotonCamera frontCamera, tapeCamera; 

    public Camera() {
        frontCamera = new PhotonCamera(VisionConstants.frontCameraName); //aprilTagCamera
        tapeCamera = new PhotonCamera(VisionConstants.tapeCameraName); //reflectiveTapeCamera
    }
    @Override
    public void periodic() {
        //Get latest result from camera. If there are targets, put the id of the "best target" defined by photonlib onto the smartdashboard. No target places -1 on dash.
        //https://docs.photonvision.org/en/latest/docs/getting-started/pipeline-tuning/reflectiveAndShape/contour-filtering.html#contour-grouping-and-sorting

        var res = frontCamera.getLatestResult();
        if (res.hasTargets()) { //if targets sighted
            var bestTarget = res.getBestTarget(); //get the "best target"
            SmartDashboard.putNumber("ID", bestTarget.getFiducialId()); //put "best target" ID onto smartdash
        }
        else {
            SmartDashboard.putNumber("ID", -1); //no target found
        }
        SmartDashboard.putNumber("Pipeline Index", frontCamera.getPipelineIndex());
        SmartDashboard.putBoolean("Has Target", tapeCamera.getLatestResult().hasTargets());
    }
    @Override
    public void simulationPeriodic() {
        
    }
    public PhotonCamera getFrontCamera() {
        //return camera object for command manipulation
        return frontCamera;
    }
    public PhotonCamera getTapeCamera() {
        return tapeCamera;
    }
}
