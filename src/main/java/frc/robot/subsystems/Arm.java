// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SimConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.robotLimit;

import java.lang.Math;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private WPI_TalonFX shoulderMotor;
	private WPI_TalonFX elbowMotor;
  private PWMSim m_simMotor;

  private final DCMotor gearbox = 
    new DCMotor(
      Constants.TalonFXConstants.nominalVoltageVolts,
      Constants.TalonFXConstants.stallTorqueNewtonMeters,
      Constants.TalonFXConstants.stallCurrentAmps,
      Constants.TalonFXConstants.freeCurrentAmps,
      Constants.TalonFXConstants.freeSpeedRadPerSec,
      1);
  private final SingleJointedArmSim m_armSim = 
    new SingleJointedArmSim(
      gearbox, 
      Constants.SimConstants.gearingShoulder, 
      Constants.SimConstants.jKgMetersSquared,
      Constants.SimConstants.totalArmLength,
      Constants.SimConstants.minAngleRads, 
      Constants.SimConstants.maxAngleRads, 
      Constants.SimConstants.armMassKg, 
      true);

  private double shoulderAngle = 0;
  private double elbowAngle = 0;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);

  private final MechanismLigament2d m_armTower =
      m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));

  private final MechanismLigament2d m_arm =
      m_armPivot.append(
          new MechanismLigament2d(
              "Arm",
              30,
              Units.radiansToDegrees(m_armSim.getAngleRads()),
              6,
              new Color8Bit(Color.kYellow)));


  public Arm() {
    shoulderMotor = new WPI_TalonFX(5);

		elbowMotor = new WPI_TalonFX(6);
    shoulderMotor.setInverted(true);
		elbowMotor.setInverted(true);
    shoulderMotor.setNeutralMode(NeutralMode.Coast);
		elbowMotor.setNeutralMode(NeutralMode.Brake);

    m_simMotor = new PWMSim(Constants.DriveConstants.kLeftMotor1Port);
  SmartDashboard.putData("Mech2d", m_mech2d);

  }

  public void reachGoal(double goal) {
    /*m_controller.setGoal(goal);

    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(m_encoder.getDistance());
    double feedforwardOutput = m_feedforward.calculate(m_controller.getSetpoint().velocity);
    m_motor.setVoltage(pidOutput + feedforwardOutput);*/
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
    double thirdSide = Math.sqrt(targetX * targetX + targetY * targetY);
    if (thirdSide + ArmConstants.elbowArmLength < ArmConstants.shoulderArmLength  || 
    thirdSide > ArmConstants.elbowArmLength + ArmConstants.shoulderArmLength) angles[2] = -1;
    else{
     angles[2] = 1;
      double oppositeElbowAngle = Math.acos((thirdSide * thirdSide + 
      ArmConstants.shoulderArmLength * ArmConstants.shoulderArmLength - 
      ArmConstants.elbowArmLength * ArmConstants.elbowArmLength) / 2 / ArmConstants.shoulderArmLength / 
      thirdSide);
      angles[1] = Math.asin(thirdSide * Math.sin(oppositeElbowAngle) / ArmConstants.elbowArmLength);
      if (thirdSide * thirdSide > ArmConstants.shoulderArmLength * ArmConstants.shoulderArmLength + 
        ArmConstants.elbowArmLength + ArmConstants.elbowArmLength){
        angles[1] = Math.PI - angles[1];
      }
      
      angles[0] = Math.abs(Math.PI/2 - oppositeElbowAngle - Math.atan(targetY/Math.abs(targetX)));
      if (targetX > 0) angles[0] = -angles[0];
      if (angles[0] > 0){
        angles[1] = -angles[1];
      }
      return angles;
    }   
    return angles;
  }

  public double[] getXY (){
    double[] xy = new double[2];
      double thirdSide = Math.sqrt(
      ArmConstants.shoulderArmLength * ArmConstants.shoulderArmLength + 
      ArmConstants.elbowArmLength * ArmConstants.elbowArmLength - 
      2 * ArmConstants.elbowArmLength * ArmConstants.shoulderArmLength * 
      Math.cos(elbowAngle)
     );
     double oppositeElbowAngle = elbowAngle * ArmConstants.elbowArmLength / 
      ArmConstants.shoulderArmLength;
    double thirdSideHorizontalAngle = Math.PI / 2 - Math.abs(shoulderAngle) - oppositeElbowAngle;
    xy[1] = thirdSide * Math.sin(thirdSideHorizontalAngle);
    xy[0] = thirdSide * Math.cos(thirdSideHorizontalAngle);
    return xy;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double elbowTicks = elbowMotor.getSelectedSensorPosition();
    double shoulderTicks = shoulderMotor.getSelectedSensorPosition();
    elbowAngle = elbowTicks * ArmConstants.elbowperMotorTick;
    shoulderAngle = shoulderTicks * ArmConstants.shoulderperMotorTick;
    SmartDashboard.putNumber("ElbowEncoder", elbowTicks);
		SmartDashboard.putNumber("ShoulderEncoder", shoulderTicks);
    SmartDashboard.putNumber("currentShoulderAngle", shoulderAngle);
  }

  public void run(double shoulderSpeed, double elbowSpeed){
      elbowMotor.set(elbowSpeed);
      shoulderMotor.set(shoulderSpeed);
  }
  public double getElbowAngle(){
    return elbowAngle;
  }
  public double getShoulderAngle(){
    return shoulderAngle;
  }
}
