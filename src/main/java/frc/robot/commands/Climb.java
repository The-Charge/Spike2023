// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class Climb extends CommandBase {
  private double powerLimit = 0.2;
  private double lastPitch = 0;
  private boolean yDropped = false;
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
  public void initialize() {
    lastPitch = m_drivetrain.getPitch();
    m_drivetrain.initializeMotors();
    powerLimit = 0.13;
    yDropped = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed, rightSpeed;
    double thisPitch = m_drivetrain.getPitch();
    leftSpeed = thisPitch*0.01 + (thisPitch-lastPitch)*0.15;
    lastPitch = thisPitch;
    if(leftSpeed < -0.01){
      leftSpeed = leftSpeed - 0.08;
    }else if (leftSpeed > 0.01){
      leftSpeed = leftSpeed + 0.06;
    }
    if(leftSpeed > powerLimit){
      leftSpeed = powerLimit;
    }
    if (leftSpeed < -powerLimit) {
      leftSpeed = -powerLimit;
    }
    /*double yAccel = m_drivetrain.getYAcceleration();
    if (Math.abs(yAccel) > 0.25){
      if (!yDropped){
        if (Math.abs(yAccel) > 0.7){
          yDropped = true;
          powerLimit = 0.12;
        } 
      }
    } */
    rightSpeed = leftSpeed;
    SmartDashboard.putNumber("power limit", powerLimit);
    
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
