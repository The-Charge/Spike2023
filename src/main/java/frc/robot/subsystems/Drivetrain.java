// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase { 

private WPI_TalonFX leftFrontMotor;
private WPI_TalonFX leftBackMotor;
private WPI_TalonFX rightFrontMotor;
private WPI_TalonFX rightBackMotor;


private MotorControllerGroup m_leftMotors; 
private MotorControllerGroup m_rightMotors; 

private DifferentialDrive differentialDrive;

public Drivetrain() {
  leftFrontMotor = new WPI_TalonFX(1);
  leftBackMotor = new WPI_TalonFX(2);
  rightFrontMotor = new WPI_TalonFX(3);
  rightBackMotor = new WPI_TalonFX(4);
  m_leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
	m_rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
  differentialDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void run(double l, double r) {
		leftFrontMotor.set(l);
		rightFrontMotor.set(r);
	}
  public void runArcade(double f, double r) {
    differentialDrive.arcadeDrive(f, r);
  }

  public void runVelocityMode(double leftVelo, double rightVelo){
    leftFrontMotor.set(ControlMode.Velocity, leftVelo);
    rightFrontMotor.set(ControlMode.Velocity, rightVelo);
  }
  //alternate velocityMode control utiliziing WPI differential drivetrain kinematics
  public void runVelocityMode(ChassisSpeeds chassisSpeeds){
    DifferentialDriveWheelSpeeds speeds = DriveConstants.kDriveKinematics.toWheelSpeeds(chassisSpeeds);
    leftFrontMotor.set(ControlMode.Velocity, speeds.leftMetersPerSecond);
    rightFrontMotor.set(ControlMode.Velocity, speeds.rightMetersPerSecond);
  }
  
  public void stop() {
		leftFrontMotor.set(ControlMode.PercentOutput, 0);
		rightFrontMotor.set(ControlMode.PercentOutput, 0);
	}

  public void setBrakeMode() {
      leftFrontMotor.setNeutralMode(NeutralMode.Brake);
      rightFrontMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setCoastMode() {
      leftFrontMotor.setNeutralMode(NeutralMode.Coast);
      rightFrontMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void initializeMotors() {

		leftFrontMotor.configFactoryDefault();
		rightFrontMotor.configFactoryDefault();

		rightBackMotor.setInverted(true);
		rightFrontMotor.setInverted(true);

		rightBackMotor.follow(rightFrontMotor);
		leftBackMotor.follow(leftFrontMotor);

		rightBackMotor.configOpenloopRamp(0.5);
		rightFrontMotor.configOpenloopRamp(0.5);
		leftBackMotor.configOpenloopRamp(0.5);
		leftFrontMotor.configOpenloopRamp(0.5);

		leftFrontMotor.configNeutralDeadband(0.08);
		rightFrontMotor.configNeutralDeadband(0.08);
  }
}
