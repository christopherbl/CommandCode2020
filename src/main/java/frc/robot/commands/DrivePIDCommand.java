/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDDriveTrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DrivePIDCommand extends CommandBase {
  /**
   * Creates a new DrivePIDCommand - this leverages example from GearsBot (Elevator (PID)subsys and 
   * SetElevatorSetpoint in WPILIB examples.
   */
  private final PIDDriveTrain m_piddrivetrain;
  int number_calls;
  private final DoubleSupplier m_forward;

  public DrivePIDCommand(PIDDriveTrain piddrivetrain, final DoubleSupplier forward) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_piddrivetrain = piddrivetrain;
    m_forward = forward;
    number_calls = 0;
  }


// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_piddrivetrain.setSetpoint(-75.0);
    m_piddrivetrain.resetPIDDriveEncoderCount();
    m_piddrivetrain.enable();
    number_calls = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_piddrivetrain.increasePIDDriveEncoderCount(m_forward.getAsDouble());
    SmartDashboard.putNumber("pidSnapshotdriveEncoderValue", m_piddrivetrain.getPIDDriveEncoderCount());
    SmartDashboard.putNumber("pidError", m_piddrivetrain.getController().getPositionError());
    SmartDashboard.putNumber("pidSetPoint", m_piddrivetrain.getController().getSetpoint());
    SmartDashboard.putNumber("pidPeriod", m_piddrivetrain.getController().getPeriod());
    SmartDashboard.putNumber("PID SECS TO CONVERGE",Double.valueOf(number_calls)*.020); 
    number_calls++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_piddrivetrain.getController().atSetpoint(); //false;
  }
}
