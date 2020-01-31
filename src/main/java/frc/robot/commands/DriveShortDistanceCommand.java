/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveShortDistanceCommand extends CommandBase {
  private final DriveTrain m_drivetrain;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotation;
  private final double m_distance_to_travel_in_feet;

  int number_calls;


  public DriveShortDistanceCommand(final DriveTrain subsystem, final DoubleSupplier forward,
      final DoubleSupplier rotation, double feet) {
    m_drivetrain = subsystem;
    m_forward = forward;
    m_rotation = rotation;
    m_distance_to_travel_in_feet = feet;
 //   number_calls = 0;
    addRequirements(m_drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetDriveEncoderCount();
  }

  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_forward.getAsDouble(), m_rotation.getAsDouble());
    m_drivetrain.increaseDriveEncoderCount(m_forward.getAsDouble());
    //m_drivetrain.incrementDriveEncoderCount();
 //   SmartDashboard.putNumber("DriveShort CALLS",number_calls);
    SmartDashboard.putNumber("ABS(EncoderDistance)",Math.abs(m_drivetrain.getDriveEncoderDistance()));
    SmartDashboard.putNumber("To Travel",m_distance_to_travel_in_feet);
 //   number_calls++;

  }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if (Math.abs(m_drivetrain.getDriveEncoderDistance()) > m_distance_to_travel_in_feet)
        return true; //(;
      else
        return false;
    }
}
