/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PIDDriveTrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ResetEncoderCommand extends CommandBase {
  private final DriveTrain m_drivetrain;
  private final PIDDriveTrain m_piddrivetrain;

  public ResetEncoderCommand(DriveTrain subsystem1, PIDDriveTrain subsystem2) {
    m_drivetrain = subsystem1;
    m_piddrivetrain = subsystem2;

    addRequirements(m_drivetrain);
  }

  @Override
  public void execute() {
    m_drivetrain.resetDriveEncoderCount();
    m_piddrivetrain.resetPIDDriveEncoderCount();
    //SmartDashboard.putNumber("ButtonSnapshotdriveEncoderValue", m_drivetrain.getDriveEncoderDistance());
  }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return true;
    }
}
