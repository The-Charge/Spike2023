// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI; 
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonFX leftFrontMotor;
  private WPI_TalonFX leftBackMotor;
  private WPI_TalonFX rightFrontMotor;
  private WPI_TalonFX rightBackMotor;

  public boolean InvertSpeed = false; 

  private AHRS m_gyro;
  private double pitch = 0;
  private double gyroOffset = 0;
  private double pitchOffset = 0;
 // public final DifferentialDriveOdometry m_odometry;


  public Drivetrain() {
    leftFrontMotor = new WPI_TalonFX(1);
    leftBackMotor = new WPI_TalonFX(2);
    rightFrontMotor = new WPI_TalonFX(3);
    rightBackMotor = new WPI_TalonFX(4);
    try { m_gyro = new AHRS(SPI.Port.kMXP);} catch (RuntimeException ex ) {DriverStation.reportError( ex.getMessage(), true);} 
    Timer.delay(1.0);  
    //m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 0, 0, new Pose2d(0, 0, new Rotation2d()));
    resetHeading();
    resetPitch();
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
    pitch = m_gyro.getPitch() - pitchOffset;
    SmartDashboard.putNumber("IMU_Pitch", pitch);
          // m_odometry.update(Rotation2d.fromDegrees(getHeading()),
        //leftFrontMotor.getSelectedSensorPosition(0) * DriveConstants.kEncoderDistancePerPulse,
       // rightFrontMotor.getSelectedSensorPosition(0) * DriveConstants.kEncoderDistancePerPulse
      //);
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
  
  public void setBrakeMode() {
    leftFrontMotor.setNeutralMode(NeutralMode.Brake);
    rightFrontMotor.setNeutralMode(NeutralMode.Brake);
  }
  public void setNeutralMode() {
    leftFrontMotor.setNeutralMode(NeutralMode.Coast);
    rightFrontMotor.setNeutralMode(NeutralMode.Coast);
  }
  
  public double getPitch(){ return pitch;}

  public void resetPitch(){ pitchOffset = m_gyro.getPitch();}

  public double getHeading() { return m_gyro.getRotation2d().getDegrees() - gyroOffset;}

  public void resetHeading() { gyroOffset = m_gyro.getRotation2d().getDegrees();}

  public void isReversed(){ InvertSpeed = !InvertSpeed;}

  public double getLeftEncoder(){ return leftFrontMotor.getSelectedSensorPosition(0);}

  public void stop() {
		leftFrontMotor.set(ControlMode.PercentOutput, 0);
		rightFrontMotor.set(ControlMode.PercentOutput, 0);
	}
}
