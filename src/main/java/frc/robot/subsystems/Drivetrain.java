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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port; //might change to I2C
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

private final AHRS m_gyro = new AHRS(Port.kMXP);
private double gyroOffset = 0;

public final DifferentialDriveOdometry m_odometry;

private static boolean isReversed = false;
private boolean halfSpeed = false;
private boolean quarterSpeed = false;
public DifferentialDriveKinematics KINEMATICS;

private static final double MM_VEL = 4000;
private static final double MM_ACC = 2000;

private final static int kTimeoutMs = 30;

private final static double kNeutralDeadband = 0.001;

private final static int kEncoderUnitsPerRotation = 51711;

private static final double TICKSPERFEET = 14312;

private final static int PID_PRIMARY = 0;
private final static int PID_TURN = 1;
private final static int kSlot_Distance = 0;
private final static int kSlot_Turning = 1;

private static final double DISTANCE_kP = 0.1;
private static final double DISTANCE_kI = 0.0;
private static final double DISTANCE_kD = 0.0;
private static final double DISTANCE_kF = 0.0;
private static final double DISTANCE_kIzone = 100;
private static final double DISTANCE_PEAK = 0.50;

private static final double TURN_kP = 0.2;
private static final double TURN_kI = 0.0;
private static final double TURN_kD = 0.0;
private static final double TURN_kF = 0.0;
private static final double TURN_kIzone = 200;
private static final double TURN_PEAK = 1.00;

private static final int PID_SLOT_SPEED_MODE = 2;
private static final double SPEED_P_CONSTANT = .2;
private static final double SPEED_I_CONSTANT = 0.0;
private static final double SPEED_D_CONSTANT = 2.0;
private static final double SPEED_F_CONSTANT = 0.0;

private static final double MAX_VELOCITY = 20000;

private static double m_distance;
private static final double THRESHOLD = 200;
private static final double MAX_TEMP = 35;

private static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(DriveConstants.kTrackwidthMeters));

public Drivetrain() {
  leftFrontMotor = new WPI_TalonFX(1);
  leftBackMotor = new WPI_TalonFX(2);
  rightFrontMotor = new WPI_TalonFX(3);
  rightBackMotor = new WPI_TalonFX(4);
  m_leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
	m_rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

  m_odometry = new DifferentialDriveOdometry(getGyroAngle(), 0, 0);

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LeftEnc", leftFrontMotor.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("RightEnc", rightFrontMotor.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("POSE X", getPose().getX());
    SmartDashboard.putNumber("POSE Y", getPose().getY());

    m_odometry.update(Rotation2d.fromDegrees(getHeading()),
            leftFrontMotor.getSelectedSensorPosition(0) * DriveConstants.kEncoderDistancePerPulse,
            rightFrontMotor.getSelectedSensorPosition(0) * DriveConstants.kEncoderDistancePerPulse);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  public void run(double l, double r) {
		leftFrontMotor.set(l);
		rightFrontMotor.set(r);
	}

  public void runVelocityMode(ChassisSpeeds chassisSpeeds){
    DifferentialDriveWheelSpeeds speeds = kinematics.toWheelSpeeds(chassisSpeeds);
    leftFrontMotor.set(ControlMode.Velocity, speeds.leftMetersPerSecond);
    rightFrontMotor.set(ControlMode.Velocity, speeds.rightMetersPerSecond);
  }
  public void runVelocityMode(double leftVelo, double rightVelo){
    leftFrontMotor.set(ControlMode.Velocity, leftVelo);
    rightFrontMotor.set(ControlMode.Velocity, rightVelo);
  }

  public void stop() {
		leftFrontMotor.set(ControlMode.PercentOutput, 0);
		rightFrontMotor.set(ControlMode.PercentOutput, 0);
	}

  public double getHeading() {
		return m_gyro.getRotation2d().getDegrees() - gyroOffset;
	}

  public Rotation2d getGyroAngle() {
    return m_gyro.getRotation2d();
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
	public Pose2d getPose() {
    return m_odometry.getPoseMeters();
}

/**
 * Returns the current wheel speeds of the robot.
 *
 * @return The current wheel speeds.
 */
public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
            leftFrontMotor.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse,
            rightFrontMotor.getSelectedSensorVelocity() * DriveConstants.kEncoderDistancePerPulse);
}
public void resetEncoders() {
  leftFrontMotor.setSelectedSensorPosition(0);
  rightFrontMotor.setSelectedSensorPosition(0);
}
/**
 * Resets the odometry to the specified pose.
 *
 * @param pose2d The pose to which to set the odometry.
 */
public void resetOdometry(edu.wpi.first.math.geometry.Pose2d pose2d) {
    resetEncoders();
    m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()), getLeftEncoder(), getRightEncoder(), pose2d);
}

/**
 * Controls the left and right sides of the drive directly with `s.
 *
 * @param leftVolts  the commanded left output
 * @param rightVolts the commanded right output
 */
public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    SmartDashboard.putNumber("Left Encoder", leftFrontMotor.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Right Encoder", rightFrontMotor.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("HEADING", m_odometry.getPoseMeters().getRotation().getDegrees());
    
}

/**
 * Resets the drive encoders to currently read a position of 0.
 */

public void setEncoders(int left, int right) {
    leftFrontMotor.setSelectedSensorPosition(left);
    rightFrontMotor.setSelectedSensorPosition(right);
}

/**
 * Zeroes the heading of the robot.
 */
public void zeroHeading() {
    //m_gyro.reset();
    gyroOffset = m_gyro.getAngle();
}
public Rotation2d getGyroRotation2d() {
  return m_gyro.getRotation2d();
}
/**
 * Returns the heading of the robot.
 *
 * @return the robot's heading in degrees, from -180 to 180
 */


public double getLeftEncoder() {
return leftFrontMotor.getSelectedSensorPosition();
}

public double getRightEncoder() {
return rightFrontMotor.getSelectedSensorPosition();
}

public boolean isAtPIDDestination() {
return (Math.abs(m_distance) - Math.abs(getLeftEncoder()) < THRESHOLD
    || Math.abs(m_distance) - Math.abs(getRightEncoder()) < THRESHOLD);
}

public boolean checkTemp() {
    if (rightFrontMotor.getTemperature() > MAX_TEMP || rightBackMotor.getTemperature() > MAX_TEMP 
|| leftFrontMotor.getTemperature() > MAX_TEMP || leftBackMotor.getTemperature() > MAX_TEMP)
        return true;
    else
        return false;
}

public double getGyroZ(){
SmartDashboard.putNumber("ZAxis", m_gyro.getWorldLinearAccelZ());
return m_gyro.getWorldLinearAccelZ();
}

public double getPitch(){
return m_gyro.getPitch();
}

public void initSpeedMode() {
  leftFrontMotor.config_kP(PID_SLOT_SPEED_MODE, SPEED_P_CONSTANT);
        leftFrontMotor.config_kI(PID_SLOT_SPEED_MODE, SPEED_I_CONSTANT);
        leftFrontMotor.config_kD(PID_SLOT_SPEED_MODE, SPEED_D_CONSTANT);
        leftFrontMotor.config_kF(PID_SLOT_SPEED_MODE, SPEED_F_CONSTANT);
        leftFrontMotor.selectProfileSlot(PID_SLOT_SPEED_MODE, 0);

		rightFrontMotor.config_kP(PID_SLOT_SPEED_MODE, SPEED_P_CONSTANT);
        rightFrontMotor.config_kI(PID_SLOT_SPEED_MODE, SPEED_I_CONSTANT);
        rightFrontMotor.config_kD(PID_SLOT_SPEED_MODE, SPEED_D_CONSTANT);
        rightFrontMotor.config_kF(PID_SLOT_SPEED_MODE, SPEED_F_CONSTANT);
    	rightFrontMotor.selectProfileSlot(PID_SLOT_SPEED_MODE, 0);

        leftFrontMotor.set(ControlMode.Velocity, 0);
		rightFrontMotor.set(ControlMode.Velocity, 0);
}

}
