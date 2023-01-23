package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.*;



public class TagAlign extends CommandBase {
    private final Drivetrain m_drivetrain;
    private final Camera m_camera;

    final double TARGET_HEIGHT_METERS = 0;
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);
    public boolean atDestination = false;

    private final PIDController forwardController = new PIDController(DriveConstants.LINEAR_P, 0, DriveConstants.LINEAR_D);
    private final PIDController turnController = new PIDController(DriveConstants.ANGULAR_P, 0, DriveConstants.ANGULAR_D);

    public TagAlign(Drivetrain m_drivetrain, Camera m_camera){
        this.m_drivetrain = m_drivetrain;
        this.m_camera = m_camera;
    }
    @Override
    public void execute() {
        double forwardSpeed;
        double rotationSpeed;
        var result = m_camera.getFrontCamera().getLatestResult();
        if (result.hasTargets()) {
            double range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS, VisionConstants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
            forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
            rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);  
            m_drivetrain.runArcade(forwardSpeed, range);
            if ((forwardSpeed == 0) && (rotationSpeed == 0)) { 
                atDestination = true;
            }
            else {
                atDestination = false;
            }
        }
        else {
            forwardSpeed = -RobotContainer.getInstance().getrightJoystick().getY();
            rotationSpeed = -RobotContainer.getInstance().getleftJoystick().getY();
            m_drivetrain.run(forwardSpeed, rotationSpeed);
        }
        SmartDashboard.putBoolean("atAprilTag", atDestination);
    }


}
