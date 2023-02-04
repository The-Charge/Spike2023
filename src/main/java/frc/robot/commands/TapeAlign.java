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

public class TapeAlign extends CommandBase{
    private final Drivetrain m_drivetrain;
    private final Camera m_camera;
    public double TARGET_HEIGHT_METERS = Units.inchesToMeters(24.5); //Height of target (to be changed)
    final double GOAL_RANGE_METERS = Units.feetToMeters(3); //distance to reach between tag and robot
    public double range, bestTargetYaw;

    //PIDControllers from example: adjust as needed
    private final PIDController forwardController = new PIDController(DriveConstants.LINEAR_P, 0, DriveConstants.LINEAR_D);
    private final PIDController turnController = new PIDController(DriveConstants.ANGULAR_P, 0, DriveConstants.ANGULAR_D);

    public TapeAlign(Drivetrain m_drivetrain, Camera m_camera){
        this.m_drivetrain = m_drivetrain;
        this.m_camera = m_camera;
        SmartDashboard.putNumber("Goal Distance", GOAL_RANGE_METERS);
        m_drivetrain.setBrakeMode();
    }
    @Override
    public void execute() {
        //speeds for arcadedrive
        double forwardSpeed, rotationSpeed;

        //get result, if there are targets within the cameraview
        var result = m_camera.getTapeCamera().getLatestResult();
        if (result.hasTargets()) {
            //calculate range between robot and bestTarget
            range = PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.TAPECAMERA_HEIGHT_METERS,
            TARGET_HEIGHT_METERS, VisionConstants.TAPECAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
            SmartDashboard.putNumber("Range", range);
            //calculate and feed speed values into runArcade in Drivetrain
            forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
            rotationSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0);
            bestTargetYaw = Math.abs(result.getBestTarget().getYaw());
            SmartDashboard.putNumber("forward", forwardSpeed);
            SmartDashboard.putNumber("Rotation", rotationSpeed);
            SmartDashboard.putNumber("Target Yaw",  result.getBestTarget().getYaw());
            m_drivetrain.runArcade(forwardSpeed, rotationSpeed);
        }
        else {
            //no targets -> run tankDrive until there is a target in sight
            forwardSpeed = -RobotContainer.getInstance().getRightJoystick().getY();
            rotationSpeed = -RobotContainer.getInstance().getLeftJoystick().getY();
            m_drivetrain.run(forwardSpeed, rotationSpeed);
        }
        SmartDashboard.putBoolean("TapeAlign Finished", isFinished());
    }
    @Override
    public boolean isFinished() {
        if (Math.abs(GOAL_RANGE_METERS - range) < 0.05 && bestTargetYaw < 2) {
            //m_drivetrain.setCoastMode();
            return true;
        }
        return false;
    }




}
