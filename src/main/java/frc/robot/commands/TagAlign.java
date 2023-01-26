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

    final double TARGET_HEIGHT_METERS = 0; //Height of target (to be changed)
    final double GOAL_RANGE_METERS = Units.feetToMeters(3); //distance to reach between tag and robot
    public boolean atDestination = false; //Robot has reached destination?

    //PIDControllers from example: adjust as needed
    private final PIDController forwardController = new PIDController(DriveConstants.LINEAR_P, 0, DriveConstants.LINEAR_D);
    private final PIDController turnController = new PIDController(DriveConstants.ANGULAR_P, 0, DriveConstants.ANGULAR_D);

    public TagAlign(Drivetrain m_drivetrain, Camera m_camera){
        this.m_drivetrain = m_drivetrain;
        this.m_camera = m_camera;
    }
    @Override
    public void execute() {
        //speeds for arcadedrive
        double forwardSpeed;
        double rotationSpeed;

        //get result, if there are targets within the cameraview
        var result = m_camera.getFrontCamera().getLatestResult();
        if (result.hasTargets()) {
            //calculate range between robot and bestTarget
            double range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS, VisionConstants.CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
            //calculate and feed speed values into runArcade in Drivetrain
            forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
            rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);  
            m_drivetrain.runArcade(forwardSpeed, range);
            //if there is no movement, robot should be atDestination
            if ((forwardSpeed == 0) && (rotationSpeed == 0)) { 
                atDestination = true;
            }
            else {
                atDestination = false;
            }
        }
        else {
            //no targets -> run tankDrive until there is a target in sight
            forwardSpeed = -RobotContainer.getInstance().getRightJoystick().getY();
            rotationSpeed = -RobotContainer.getInstance().getLeftJoystick().getY();
            m_drivetrain.run(forwardSpeed, rotationSpeed);
        }
        SmartDashboard.putBoolean("atAprilTag", atDestination); //for testing
    }


}
