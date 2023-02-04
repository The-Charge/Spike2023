package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;

public class RobotContainer {

    private static RobotContainer m_robotContainer = new RobotContainer();

    public final Drivetrain m_drivetrain = new Drivetrain();

    public final Camera m_camera = new Camera();

    // Joysticks
    private final Joystick rightJoystick = new Joystick(1);
    private final Joystick leftJoystick = new Joystick(0);

    // A chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    RobotContainer() {

        configureButtonBindings();
        m_drivetrain.setDefaultCommand(new TankDrive( m_drivetrain ) );

        SmartDashboard.putData("AutoSelect", m_chooser);
        
    }

    public static RobotContainer getInstance() {
        return m_robotContainer;
    }
    public void configureButtonBindings() {
        JoystickButton tagAlignButton = new JoystickButton(rightJoystick, 1);
        tagAlignButton.whileTrue(new TagAlign(m_drivetrain, m_camera)); //While held
        JoystickButton tapeAlignButton = new JoystickButton(leftJoystick, 1);
        tapeAlignButton.whileTrue(new TapeAlign(m_drivetrain, m_camera)); //While held

    }

    public Joystick getLeftJoystick() {
        return leftJoystick;
    }

    public Joystick getRightJoystick() {
        return rightJoystick;
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // The selected command will be run in autonomous
        return m_chooser.getSelected();
    }

}