// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
  /** Creates a new MoveArm. */
  private Arm m_arm;
  private double targetElbowAngle = 0;
  private double targetShoulderAngle = 0;
  private double targetX = ArmConstants.targetX[0];
  private double targetY = ArmConstants.targetY[0];
  private boolean inTransittion = false;
  private int armState = 0;
  // 0: neutral; 1: 1st quad pickup; 2: 1st quad score; 3: 2nd quad pickup; 4: 2nd quad score
  private boolean secondStepNeeded = false;
  private int finalState = 0;
  private boolean elbowHitIntermediate = false;
  private boolean shoulderHitTarget = false;
  
  public MoveArm(Arm subsystem) {
    m_arm = subsystem;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armState = 0;
    targetX = ArmConstants.targetX[armState];
    targetY = ArmConstants.targetY[armState];
    inTransittion = false;
    secondStepNeeded = false;
    targetElbowAngle = 0;
    targetShoulderAngle = 0;
    elbowHitIntermediate = true;
    shoulderHitTarget = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("armState", armState);
    SmartDashboard.putBoolean("Elbowhit", elbowHitIntermediate);
    if (inTransittion || secondStepNeeded){
      if(!inTransittion){
        armState = finalState;
        inTransittion = true;
        elbowHitIntermediate = false;          
        secondStepNeeded = false;
      }
      if(elbowHitIntermediate){
        targetShoulderAngle = ArmConstants.targetShoulder[armState];
        shoulderHitTarget = Math.abs(m_arm.getShoulderAngle() - targetShoulderAngle) < 0.02;
        if(shoulderHitTarget) {
          targetElbowAngle = ArmConstants.targetElbow[armState];
          if (Math.abs(m_arm.getElbowAngle() - targetElbowAngle) < 0.02){
            inTransittion = false;
            targetX = ArmConstants.targetX[armState];
            targetY = ArmConstants.targetY[armState];
          }
        }
      }else{
        if(armState == 1 || armState == 2)targetElbowAngle = Math.PI/2;
        else if (armState == 3 ||armState == 4)targetElbowAngle = -Math.PI/2;
        else if (m_arm.getShoulderAngle() < 0)targetElbowAngle = Math.PI/2;
        else targetElbowAngle = -Math.PI/2;
        elbowHitIntermediate = Math.abs(m_arm.getElbowAngle() - targetElbowAngle) < 0.02;
      }
    }else{
      double xSpeed = -RobotContainer.getInstance().getleftJoystick().getY()/100;
      double ySpeed = -RobotContainer.getInstance().getrightJoystick().getY()/100;
      if (Math.abs(xSpeed) < 0.0005)xSpeed = 0;
      if (Math.abs(ySpeed) < 0.0005)ySpeed = 0;
      if (armState == 0){
        targetShoulderAngle = targetShoulderAngle + xSpeed;
        targetElbowAngle = targetElbowAngle + xSpeed;
        if(targetShoulderAngle > .1){
          targetShoulderAngle = 0.1;
        }else if (targetShoulderAngle < -.1){
          targetShoulderAngle = -.1;
        }
        if (targetElbowAngle > .2){
          targetElbowAngle = .2;
        }else if (targetElbowAngle < -.2){
          targetElbowAngle = .2;
        }
      }else {
        double oldX = targetX;
        double oldY = targetY;
        targetX = targetX + xSpeed;
        targetY = targetY + ySpeed;
        if (targetX > ArmConstants.targetX[armState] + ArmConstants.xRange[armState]){
           targetX = ArmConstants.targetX[armState] + ArmConstants.xRange[armState];
        }else if (targetX < ArmConstants.targetX[armState] -  ArmConstants.xRange[armState]){
         targetX = ArmConstants.targetX[armState] -  ArmConstants.xRange[armState];
        }
        if (targetY > ArmConstants.targetY[armState] + ArmConstants.yRange[armState]){
           targetY = ArmConstants.targetY[armState] + ArmConstants.yRange[armState];
        }else if (targetY < ArmConstants.targetY[armState] - ArmConstants.yRange[armState]){
           targetY = ArmConstants.targetY[armState] - ArmConstants.yRange[armState];
        }
        if(!m_arm.isXYLimit(targetX, targetY)){
          targetX = oldX;
          targetY = oldY;
        }
        SmartDashboard.putNumber("targetX", targetX);
        SmartDashboard.putNumber("targetY", targetY);
    
        double[] angles = m_arm.getAngles(targetX, targetY);
        if (angles[2] > 0){
          targetShoulderAngle = angles[0];
          targetElbowAngle = angles[1];
        }

        if (RobotContainer.getInstance().getleftJoystick().getRawButton(1) && !(armState == 0)){
          inTransittion = true;
          elbowHitIntermediate = false;
          armState = 0;
        }else if(RobotContainer.getInstance().getleftJoystick().getRawButton(2) && !(armState == 1)){
          inTransittion = true;
          elbowHitIntermediate = false;
          if(armState == 3 || armState == 4){
            secondStepNeeded = true;
            armState = 0;
            finalState = 1;
          }else armState = 1;
        }else if(RobotContainer.getInstance().getleftJoystick().getRawButton(3) && !(armState == 2)){
          inTransittion = true;
          elbowHitIntermediate = false;
          if(armState == 3 || armState == 4){
            secondStepNeeded = true;
            armState = 0;
            finalState = 2;
          }else armState = 2;
        }else if(RobotContainer.getInstance().getleftJoystick().getRawButton(4) && !(armState == 3)){
          inTransittion = true;
          elbowHitIntermediate = false;
          if(armState == 1 || armState == 2){
            secondStepNeeded = true;
            armState = 0;
            finalState = 3;
          }else armState = 3;
        }else if(RobotContainer.getInstance().getleftJoystick().getRawButton(5) && !(armState == 4)){
          inTransittion = true;
          elbowHitIntermediate = false;
          armState = 4;
          if(armState == 1 || armState == 2){
            secondStepNeeded = true;
            armState = 0;
            finalState = 4;
          }else armState = 4;
        }
      } 
    }
    holdPosition(); 
  }

  private void holdPosition(){
    double currentElbowAngle = m_arm.getElbowAngle();
    double currentShoulderAngle = m_arm.getShoulderAngle();
    double temp = targetShoulderAngle-currentShoulderAngle;
    double temp2 =  targetElbowAngle-currentElbowAngle;
    m_arm.run(0.1 * temp + Math.abs(temp)/temp*ArmConstants.shoulderRestVoltage[armState], 
      0.2 * (targetElbowAngle - currentElbowAngle) + ArmConstants.elbowRestVoltage[armState]);
    SmartDashboard.putNumber("targetshoulder", targetShoulderAngle);
    SmartDashboard.putNumber("targetelbow", targetElbowAngle);
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
