package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class TankDrive extends CommandBase {
  private final Drivetrain m_drivetrain;

  /**
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDrive(Drivetrain subsystem) {
    m_drivetrain = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.initializeMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed, rightSpeed;
    rightSpeed = -RobotContainer.getInstance().getrightJoystick().getY();
    leftSpeed = -RobotContainer.getInstance().getleftJoystick().getY();

    m_drivetrain.run(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
