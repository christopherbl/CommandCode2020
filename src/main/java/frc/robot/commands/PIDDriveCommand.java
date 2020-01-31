/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PIDDriveCommand extends PIDCommand {
  private static final double SETPOINT_DISTANCE = 175.0;  // Set reference to target
  
  /**
   * Creates a new PIDDriveCommand - this is new 2020 approach but could not get dashboard to show anything but
   * zero setpoint (bug?).
   */
  private final DriveTrain m_drivetrain;
  int number_calls;

  public PIDDriveCommand(DriveTrain subsystem) {
    // super(
    //     // The controller that the command will use
    //     new PIDController(0, 0, 0),
    //     // This should return the measurement
    //     () -> 0,
    //     // This should return the setpoint (can also be a constant)
    //     () -> 0,
    //     // This uses the output
    //     output -> {
    //       // Use the output here
    //     });
   
    super(
        new PIDController(Constants.kDriveP,Constants.kDriveI,Constants.kDriveD),
          // Close loop on heading
        subsystem::getDriveEncoderDistance,
        SETPOINT_DISTANCE,
          // Pipe output to move robot
        output -> subsystem.arcadeDrive(output, 0),
          // Requires the drive subsystem
        subsystem);

        m_drivetrain = subsystem;
        number_calls = 0;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  
  @Override
  public void execute() {
    // SmartDashboard.putNumber("pidSnapshotdriveEncoderValue", m_drivetrain.getDriveEncoderCount());
    // SmartDashboard.putNumber("pidError", this.getController().getPositionError());
    // SmartDashboard.putNumber("pidSetPoint", this.getController().getSetpoint());
    // SmartDashboard.putNumber("pidPeriod", this.getController().getPeriod());
    // SmartDashboard.getBoolean("pidSetPoint", this.isScheduled());
    // SmartDashboard.putNumber("PID CALLS",number_calls);

//    m_drivetrain.increaseDriveEncoderCount(m_forward.getAsDouble());
    //m_drivetrain.incrementDriveEncoderCount();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //getController().atSetpoint();
  }
}
