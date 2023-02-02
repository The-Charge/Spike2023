// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
  private boolean isOverLimit(double targetShoulderAngle, double targetElbowAngle){
    double thirdSide = Math.sqrt(
      Constants.ArmConstants.shoulderArmLengthInch * Constants.ArmConstants.shoulderArmLengthInch + 
      Constants.ArmConstants.elbowArmLengthInch * Constants.ArmConstants.elbowArmLengthInch - 
      2 * Constants.ArmConstants.elbowArmLengthInch * Constants.ArmConstants.shoulderArmLengthInch * 
      Math.cos(targetElbowAngle)
     );
     double oppositeElbowAngle = targetElbowAngle * Constants.ArmConstants.elbowArmLengthInch / 
      Constants.ArmConstants.shoulderArmLengthInch;
    double thirdSideHorizontalAngle = Math.PI / 2 - Math.abs(targetShoulderAngle) - oppositeElbowAngle;
    double y = thirdSide * Math.sin(thirdSideHorizontalAngle);
    double x = thirdSide * Math.cos(thirdSideHorizontalAngle);
    if (targetElbowAngle > 0){
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
    SmartDashboard.putNumber("ElbowEncoder", elbowMotor.getSelectedSensorPosition());
		SmartDashboard.putNumber("ShouldrEncoder", shoulderMotor.getSelectedSensorPosition());
  }
  
  public void run(double targetElbowAngle, double targetShoulderAngle){
    if (isOverLimit(targetShoulderAngle, targetElbowAngle)){
      SmartDashboard.putBoolean("ArmTargetisOverLimit", true);
    }else {
      SmartDashboard.putBoolean("ArmTargetisOverLimit", false);
      elbowMotor.set(targetElbowAngle);
      shoulderMotor.set(targetShoulderAngle);
    }
  }
}
