/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PIDDriveTrain;

public class InitAndRunPIDDrive extends SequentialCommandGroup {
  /**
   * Create a new command group.
   *
   */
  public InitAndRunPIDDrive(DriveTrain drivetrain,PIDDriveTrain piddrivetrain, Joystick driverController) {
    addCommands(
        new ResetEncoderCommand(drivetrain,piddrivetrain),
        new DrivePIDCommand(piddrivetrain, null));
  }
}
