// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class DriveForward extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_drivetrain;
  private double m_speed; 
  private final double m_stopPitch;
  private double startTick = 0;
  private boolean isTimeMode = false;
  private double thisPitch = 0;
  /**
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForward(Drivetrain subsystem, double speed, double stopPitch) {
    m_drivetrain = subsystem;
    m_speed = speed;
    m_stopPitch = stopPitch;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isTimeMode = false;
    m_drivetrain.initializeMotors();
    m_drivetrain.setNeutralMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    thisPitch = m_drivetrain.getPitch();
    double thisHeading = m_drivetrain.getHeading() * AutoConstants.headingGain;
    if (Math.abs(thisPitch) > m_stopPitch && !isTimeMode){
      startTick = m_drivetrain.getLeftEncoder();
      isTimeMode = true;
    }
    m_drivetrain.run(m_speed + thisHeading, m_speed - thisHeading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {m_drivetrain.stop();}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isTimeMode && 
        ((Math.abs(m_drivetrain.getLeftEncoder() - startTick) > AutoConstants.fastClimbTicks) 
         || (Math.abs(thisPitch) < Math.abs(m_stopPitch)/2))){
      m_drivetrain.run(0,0);
      return true;
    }
    return false;
  }
}
