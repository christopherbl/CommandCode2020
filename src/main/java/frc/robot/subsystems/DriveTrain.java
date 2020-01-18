/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {

  private final SpeedControllerGroup m_left =
      new SpeedControllerGroup(new WPI_TalonSRX(Constants.DRIVETRAIN_FRONT_LEFT_TALON),
                               new WPI_TalonSRX(Constants.DRIVETRAIN_REAR_LEFT_TALON));

  private final SpeedControllerGroup m_right =
      new SpeedControllerGroup(new WPI_TalonSRX(Constants.DRIVETRAIN_FRONT_RIGHT_TALON),
                               new WPI_TalonSRX(Constants.DRIVETRAIN_REAR_RIGHT_TALON));

  private final DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  public DriveTrain() {
  }

  public void arcadeDrive(double forward, double rotation) {
    m_drive.arcadeDrive(forward, rotation);
  }


}