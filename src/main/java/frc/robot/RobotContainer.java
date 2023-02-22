package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
public class RobotContainer {

    private static RobotContainer m_robotContainer = new RobotContainer();

    public final Drivetrain m_drivetrain = new Drivetrain();

    // Joysticks
    private final Joystick buttonBox = new Joystick(2);
    private final Joystick rightJoystick = new Joystick(1);
    private final Joystick leftJoystick = new Joystick(0);

    // A chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    RobotContainer() {

        //configureButtonBindings();
        m_drivetrain.setDefaultCommand(new TankDrive(m_drivetrain));

        m_chooser.addOption("DriveAndClimb", new SequentialCommandGroup(
            new DriveForward(m_drivetrain, -.3, 10), new Climb(m_drivetrain)));
        m_chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());
  
        m_chooser.addOption("DriveOver", new DriveOver(m_drivetrain, .3, 10));

        m_chooser.addOption("DriveOverBackClimb", new SequentialCommandGroup(new DriveOver(m_drivetrain, -.3, 10), 
        new DriveForward(m_drivetrain, 0.3, 10), new Climb(m_drivetrain)));
        
        SmartDashboard.putData("AutoSelect", m_chooser);
 
        SmartDashboard.putData("DriveOverBackClimb", new SequentialCommandGroup(new DriveOver(m_drivetrain, -.3, 10), 
        new DriveForward(m_drivetrain, 0.3, 10), new Climb(m_drivetrain)));

        SmartDashboard.putData("drive_climb", new SequentialCommandGroup(
            new DriveForward(m_drivetrain, -.3, 10), new Climb(m_drivetrain)));
    }

    public static RobotContainer getInstance() {
        return m_robotContainer;
    }

    public Joystick getleftJoystick() {
        return leftJoystick;
    }

    public Joystick getrightJoystick() {
        return rightJoystick;
    }

    public Joystick getbuttonBox() {
        return buttonBox;
    }

    public Command getAutonomousCommand() {
        // The selected command will be run in autonomous
        return m_chooser.getSelected();
    }
}