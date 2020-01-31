/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class EncoderCommand extends CommandBase {
  private final DriveTrain m_drivetrain;

  public EncoderCommand(DriveTrain subsystem) {
    m_drivetrain = subsystem;

    addRequirements(m_drivetrain);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("ButtonSnapshotdriveEncoderValue", m_drivetrain.getDriveEncoderCount());
  }

      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return true;
      }
}
