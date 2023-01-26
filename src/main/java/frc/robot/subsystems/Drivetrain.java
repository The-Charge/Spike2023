// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI; 
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort; //might change to I2C
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  

  private WPI_TalonFX leftFrontMotor;
  private WPI_TalonFX leftBackMotor;
  private WPI_TalonFX rightFrontMotor;
  private WPI_TalonFX rightBackMotor;


  private MotorControllerGroup m_leftMotors; 

  private MotorControllerGroup m_rightMotors;

  public boolean InvertSpeed = false; 

  private AHRS m_gyro;
  // private double gyroOffset = 0;

  public Drivetrain() {
    leftFrontMotor = new WPI_TalonFX(1);
    leftBackMotor = new WPI_TalonFX(2);
    rightFrontMotor = new WPI_TalonFX(3);
    rightBackMotor = new WPI_TalonFX(4);
    try { m_gyro = new AHRS(SPI.Port.kMXP);} catch (RuntimeException ex ) {DriverStation.reportError( ex.getMessage(), true);} 
    Timer.delay(1.0);
  }

  public CommandBase exampleMethodCommand() {
      // Inline construction of command goes here.
      // Subsystem::RunOnce implicitly requires `this` subsystem.
      return runOnce(
          () -> {
            /* one-time action goes here */
          });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
  // Query some boolean state, such as a digital sensor.
    return false;
  } 

  @Override
  public void periodic() {
      // This method will be called once per scheduler run
      SmartDashboard.putBoolean("IMU_Connected", m_gyro.isConnected());
      SmartDashboard.putNumber("IMU_Yaw", m_gyro.getYaw());
      SmartDashboard.putNumber("IMU_Pitch", m_gyro.getPitch());
      SmartDashboard.putNumber("IMU_Roll", m_gyro.getRoll());
  }

  @Override
  public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
  }

  public void run(double l, double r) {
	  leftFrontMotor.set(l);
	  rightFrontMotor.set(r);
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
  public double getPitch(){
   return m_gyro.getPitch();
  }

  public void isReversed(){
    InvertSpeed = !InvertSpeed;
  }
}
