// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.robotLimit;

import java.lang.Math;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private WPI_TalonFX shoulderMotor;
	private WPI_TalonFX elbowMotor;
  private double shoulderAngle = 0;
  private double elbowAngle = 0;


  public Arm() {
    shoulderMotor = new WPI_TalonFX(5);

		elbowMotor = new WPI_TalonFX(6);
    shoulderMotor.setInverted(true);
		elbowMotor.setInverted(true);
    shoulderMotor.setNeutralMode(NeutralMode.Brake);
		elbowMotor.setNeutralMode(NeutralMode.Brake);

  }

  public boolean isInLimit(double targetShoulderAngle, double targetElbowAngle){
    double shoulderHorizen = Math.PI/2 + targetShoulderAngle;
    double elbowHorizen = -Math.PI/2 + targetElbowAngle + targetShoulderAngle;
    double x = Math.abs(Math.cos(shoulderHorizen) * ArmConstants.shoulderArmLength +
                        Math.cos(elbowHorizen) * ArmConstants.elbowArmLength);
    double y = Math.sin(shoulderHorizen) * ArmConstants.shoulderArmLength +
               Math.sin(elbowHorizen) * ArmConstants.elbowArmLength;
    if(y + ArmConstants.shoulderHeight > robotLimit.height || 
       y < - ArmConstants.shoulderHeight) return false;
    if(x > robotLimit.widthFromCenter) return false;
    if (x < robotLimit.robotLength / 2 && y < 0) return false;
    return true;
  }

  public double[] getTorques(double endWeight){
    // Equations are from "CASE STUDIES IN DUAL-LINK ROBOT ARM DYNAMICS" by Greg Zencewicz
    // the first returned value is for shoulder torque, the second is for elbow torque
    double[] torques = new double[2];
    double shoulderHorizen = Math.PI/2 + shoulderAngle;
    torques[1] = ((endWeight + ArmConstants.elbowWeight) * ArmConstants.shoulderArmLength +
                  (endWeight + ArmConstants.elbowWeight / 2) * 
                  ArmConstants.elbowArmLength) * Math.cos(shoulderHorizen + Math.PI + elbowAngle);
    torques[0] = Math.abs((ArmConstants.shoulderWeight / 2 + ArmConstants.elbowMotorWeight) * 
                          ArmConstants.shoulderArmLength * Math.cos(shoulderHorizen) + torques[1]);
    torques[1] = Math.abs(torques[1]);
    return torques;
  }

  public boolean isShoulderWithGravity(double targetShoulderAngle){
    if (Math.abs(shoulderAngle) < .00001)return true;
    return (targetShoulderAngle / shoulderAngle > 1);
  }
  
  public boolean isElbowWithGravity (double endElbowAngle){
    double elbowAdjusted = elbowAngle + shoulderAngle;

    if (Math.abs(elbowAdjusted) < .00001) return false;
    return ((endElbowAngle + shoulderAngle) / elbowAdjusted < 1);
  }

  public double[] getAngles(double targetX, double targetY){
    // first value is for shoulder angle to achieve the target x,y
    // second value is for elbow angle to achieve the target x,y
    // third value is 1 if yes and -1 for no feasible solution.
    double[] angles = new double[3];
    return angles;
  }

  private boolean isOverLimit(double targetShoulderAngle, double targetElbowAngle){
    double thirdSide = Math.sqrt(
      ArmConstants.shoulderArmLength * ArmConstants.shoulderArmLength + 
      ArmConstants.elbowArmLength * ArmConstants.elbowArmLength - 
      2 * ArmConstants.elbowArmLength * ArmConstants.shoulderArmLength * 
      Math.cos(targetElbowAngle)
     );
     double oppositeElbowAngle = targetElbowAngle * ArmConstants.elbowArmLength / 
      ArmConstants.shoulderArmLength;
    double thirdSideHorizontalAngle = Math.PI / 2 - Math.abs(targetShoulderAngle) - oppositeElbowAngle;
    double y = thirdSide * Math.sin(thirdSideHorizontalAngle);
    double x = thirdSide * Math.cos(thirdSideHorizontalAngle);
    if (targetElbowAngle * targetShoulderAngle < 0){
      if (y > 78 || x > 66){
        return true;
      }else return false;
    }else{
      if (x > 78 || y > 66){
        return true;
      }else return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double elbowTicks = elbowMotor.getSelectedSensorPosition();
    double shoulderTicks = shoulderMotor.getSelectedSensorPosition();
    elbowAngle = elbowTicks * ArmConstants.elbowperMotorTick;
    shoulderAngle = shoulderTicks * ArmConstants.shoulderperMotorTick;
    SmartDashboard.putNumber("ElbowEncoder", elbowTicks);
		SmartDashboard.putNumber("ShouldrEncoder", shoulderTicks);
  }

  public void run(double shoulderSpeed, double elbowSpeed){
      elbowMotor.set(elbowSpeed);
      shoulderMotor.set(shoulderSpeed);
  }
}

