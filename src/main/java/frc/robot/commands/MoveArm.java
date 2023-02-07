// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  private Arm m_arm;
  private double x;
  private double y;

  public MoveArm(Arm subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = subsystem;
    addRequirements(m_arm);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x = 0;
    y = ArmConstants.shoulderArmLength - ArmConstants.elbowArmLength;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shoulderSpeed = -RobotContainer.getInstance().getrightJoystick().getY()/10;
    // double elbowSpeed = -RobotContainer.getInstance().getleftJoystick().getY();
   // if (RobotContainer.getInstance().getrightJoystick().getTriggerPressed()){
   //   double[] xy = m_arm.getXY();
   //   x = xy[0];
   //   y = xy[1];
   // }
   if (Math.abs(shoulderSpeed) < 0.05){
    holdPosition();
   }else m_arm.run(shoulderSpeed, 0);
  }

  private void holdPosition(){
    //double[] angles = m_arm.getAngles(x, y);
    //double elbowAngle = angles[1];
    //double[] torques = m_arm.getTorques(0);   
    //double currentElbowAngle = m_arm.getElbowAngle();
    double currentShoulderAngle = m_arm.getShoulderAngle();
    //if (m_arm.isElbowWithGravity(currentShoulderAngle)){
    //  m_arm.run(0, 0);
    //}else{
    //  m_arm.run(.5, .5);
    //}
    if (m_arm.isShoulderWithGravity(0.6)){
    m_arm.run(0.1 * (0.6 - currentShoulderAngle), 0);
    }else {
      m_arm.run(0.5 * (0.6 - currentShoulderAngle), 0);
    }
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
