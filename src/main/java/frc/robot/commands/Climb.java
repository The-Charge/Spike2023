// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class Climb extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_drivetrain;
  /**
   *
   * @param subsystem The subsystem used by this command.
   */
  public Climb(Drivetrain subsystem) {
    m_drivetrain = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {m_drivetrain.initializeMotors();}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed, rightSpeed;
    leftSpeed = m_drivetrain.getPitch()*0.05;
    if(leftSpeed > .15){
      leftSpeed = .15;
    }
    if(leftSpeed < -.15){
      leftSpeed = -.15;
    }
    
    rightSpeed = leftSpeed;
    
   // rightSpeed = -RobotContainer.getInstance().getrightJoystick().getY();
  //  leftSpeed = -RobotContainer.getInstance().getleftJoystick().getY();

     m_drivetrain.run(leftSpeed, rightSpeed);
     //SmartDashboard.putNumber("joystick", rightSpeed + leftSpeed);
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
