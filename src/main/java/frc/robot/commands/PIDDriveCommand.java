/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.PIDDriveTrain;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class PIDDriveCommand extends PIDCommand {
  private static final double SETPOINT_DISTANCE = 125.0;  // Set reference to target
  
  /**
   * Creates a new PIDDriveCommand - this is new 2020 approach but could not get dashboard to show anything but
   * zero setpoint.
   */
  private final PIDDriveTrain m_piddrivetrain;
  int number_calls;
  DoubleSupplier m_forward;

  public PIDDriveCommand(PIDDriveTrain subsystem, final DoubleSupplier forward) {
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
   
    // this is new 2020 PID controller
    super(
        new PIDController(Constants.PID_DRIVE_P,Constants.PID_DRIVE_I,Constants.PID_DRIVE_D),
          // Close loop on heading
        subsystem::getPIDDriveEncoderDistance,
        Constants.PID_SETPOINT_DISTANCE,
          // Pipe output to move robot
        output -> subsystem.useOutput(output, 0),
          // Requires the drive subsystem
        subsystem);

        m_piddrivetrain = subsystem;
        number_calls = 0;
        m_forward = forward;
        
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_piddrivetrain.setSetpoint(-75.0);
    m_piddrivetrain.resetPIDDriveEncoderCount();
    m_piddrivetrain.enable();
    number_calls = 0;
    System.out.println("In 2020 PID INIT");      // these go to TERMINAL in VS Code 
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

    m_piddrivetrain.increasePIDDriveEncoderCount(m_forward.getAsDouble());
    SmartDashboard.putNumber("pidSnapshotdriveEncoderValue", m_piddrivetrain.getPIDDriveEncoderCount());
    SmartDashboard.putNumber("pidError", m_piddrivetrain.getController().getPositionError());
    SmartDashboard.putNumber("pidSetPoint", m_piddrivetrain.getController().getSetpoint());
    SmartDashboard.putNumber("pidPeriod", m_piddrivetrain.getController().getPeriod());
    SmartDashboard.putNumber("PID SECS TO CONVERGE",Double.valueOf(number_calls)*.020); 
    number_calls++;
    System.out.println("In 2020 PID EXE");    // these go to TERMINAL in VS Code   
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_piddrivetrain.getController().atSetpoint()) {
    System.out.println("In 2020 PID FINISH");     // these go to TERMINAL in VS Code 
  }
    return m_piddrivetrain.getController().atSetpoint();  //return false; //getController().atSetpoint();
    
  }
}
