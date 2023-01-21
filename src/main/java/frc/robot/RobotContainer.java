// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS


    // A chooser for autonomous commands
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    RobotContainer() {

        //configureButtonBindings();
    m_drivetrain.setDefaultCommand(new TankDrive( m_drivetrain ) );


        SmartDashboard.putData("AutoSelect", m_chooser);
    }

    public static RobotContainer getInstance() {
        return m_robotContainer;
    }


// SmartDashboard.putData("resetPivots", new InstantCommand(() -> m_pivot.zeroSensors()));
// SmartDashboard.putData("resetTele", new InstantCommand(() -> m_telescope.resetTeleEncoders()));
// SmartDashboard.putData("resetDrivetrain", new InstantCommand(() -> m_drivetrain.resetEncoders()));


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=BUTTONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
public Joystick getleftJoystick() {
        return leftJoystick;
    }

public Joystick getrightJoystick() {
        return rightJoystick;
    }

public Joystick getbuttonBox() {
        return buttonBox;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

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