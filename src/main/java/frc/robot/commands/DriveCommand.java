/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends CommandBase {
  private final DriveTrain m_drivetrain;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_rotation;


  public DriveCommand(DriveTrain subsystem, DoubleSupplier forward, DoubleSupplier rotation) {
    m_drivetrain = subsystem;
    m_forward = forward;
    m_rotation = rotation;
    addRequirements(m_drivetrain);
  }

  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(m_forward.getAsDouble(), m_rotation.getAsDouble());
  }
}
