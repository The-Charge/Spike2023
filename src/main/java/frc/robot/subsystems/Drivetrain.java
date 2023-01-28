package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private WPI_TalonFX leftFrontMotor;
  private WPI_TalonFX leftBackMotor;
  private WPI_TalonFX rightFrontMotor;
  private WPI_TalonFX rightBackMotor;

  private MotorControllerGroup m_leftMotors;
  private MotorControllerGroup m_rightMotors;

  public Drivetrain() {
    leftFrontMotor = new WPI_TalonFX(1);
    leftBackMotor = new WPI_TalonFX(2);
    rightFrontMotor = new WPI_TalonFX(3);
    rightBackMotor = new WPI_TalonFX(4);
    m_leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
    m_rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);
  }

  public void initializeMotors() {
    leftFrontMotor.configFactoryDefault();
    rightFrontMotor.configFactoryDefault();

    rightFrontMotor.setInverted(true);
    rightBackMotor.setInverted(true);

    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

    leftFrontMotor.configOpenloopRamp(0.5);
    leftBackMotor.configOpenloopRamp(0.5);
    rightFrontMotor.configOpenloopRamp(0.5);
    rightBackMotor.configOpenloopRamp(0.5);

    leftFrontMotor.configNeutralDeadband(0.08);
    rightFrontMotor.configNeutralDeadband(0.08);
  }

  public void run(double l, double r) {
    leftFrontMotor.set(l);
    rightFrontMotor.set(r);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
  }

  // This method will be called once per scheduler run during simulation
  @Override
  public void simulationPeriodic() {
  }

}
