/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveTrain m_drivetrain = new DriveTrain();
  private final PIDDriveTrain m_piddrivetrain = new PIDDriveTrain();

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  Joystick m_driverController = new Joystick(Constants.OI_DRIVER_CONTROLLER);


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(new DriveCommand(m_drivetrain,() -> m_driverController.getRawAxis(Constants.OI_DRIVER_CONTROLLER_X_AXIS),() -> m_driverController.getRawAxis(Constants.OI_DRIVER_CONTROLLER_Y_AXIS)));
//    m_drivetrain.setDefaultCommand(new DriveCommand(m_drivetrain,0.5,0.0));
    SmartDashboard.putNumber("CONTROLLER_X_AXIS", m_driverController.getRawAxis(Constants.OI_DRIVER_CONTROLLER_X_AXIS));
    SmartDashboard.putNumber("CONTROLLER_Y_AXIS", m_driverController.getRawAxis(Constants.OI_DRIVER_CONTROLLER_Y_AXIS));
    SmartDashboard.putNumber("DRIVE ENCODER ABSOLUTE VALUE",Math.abs(m_drivetrain.getDriveEncoderDistance()));
  
//    SmartDashboard.putData(m_piddrivetrain);
   
    SmartDashboard.putData("TunePID",m_piddrivetrain.getController());

    // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
    CommandScheduler.getInstance().onCommandInitialize(command -> Shuffleboard.addEventMarker(
        "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance().onCommandInterrupt(command -> Shuffleboard.addEventMarker(
        "Command interrupted", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance().onCommandFinish(command -> Shuffleboard.addEventMarker(
        "Command finished", command.getName(), EventImportance.kNormal));

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton readEncoderButton = new JoystickButton(m_driverController, Constants.READ_ENCODER_BUTTON);
    readEncoderButton.whenPressed(new EncoderCommand(m_drivetrain));

    JoystickButton resetEncoderButton = new JoystickButton(m_driverController, Constants.RESET_ENCODER_BUTTON);
    resetEncoderButton.whenPressed(new ResetEncoderCommand(m_drivetrain,m_piddrivetrain));

    JoystickButton driveShortDistanceButton = new JoystickButton(m_driverController, Constants.DRIVE_SHORT_DISTANCE_BUTTON);
    driveShortDistanceButton.whenPressed(new DriveShortDistanceCommand(m_drivetrain, () -> m_driverController.getRawAxis(Constants.OI_DRIVER_CONTROLLER_X_AXIS),() -> m_driverController.getRawAxis(Constants.OI_DRIVER_CONTROLLER_Y_AXIS),Constants.SHORT_DISTANCE_TO_DRIVE_IN_FEET));

    JoystickButton drivePIDDistanceButton = new JoystickButton(m_driverController, Constants.DRIVE_PID_DISTANCE_BUTTON);
    drivePIDDistanceButton.whenPressed(new DrivePIDCommand(m_piddrivetrain, () -> m_driverController.getRawAxis(Constants.OI_DRIVER_CONTROLLER_X_AXIS)));


  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
