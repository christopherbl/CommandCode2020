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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {

  
  // private final SpeedControllerGroup m_left =
  //     new SpeedControllerGroup(new WPI_TalonSRX(Constants.DRIVETRAIN_FRONT_LEFT_TALON),
  //                              new WPI_TalonSRX(Constants.DRIVETRAIN_REAR_LEFT_TALON));

  // private final SpeedControllerGroup m_right =
  //     new SpeedControllerGroup(new WPI_TalonSRX(Constants.DRIVETRAIN_FRONT_RIGHT_TALON),
  //                              new WPI_TalonSRX(Constants.DRIVETRAIN_REAR_RIGHT_TALON));

  //uncomment if you want to see output with simulation (can WPI_TalonSRX not simulation-capable)                             
  private final SpeedControllerGroup m_left =
      new SpeedControllerGroup(new PWMTalonFX(Constants.DRIVETRAIN_FRONT_LEFT_TALON),
                              new PWMTalonFX(Constants.DRIVETRAIN_REAR_LEFT_TALON));
                         
  private final SpeedControllerGroup m_right =
      new SpeedControllerGroup(new PWMTalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_TALON),
                              new PWMTalonFX(Constants.DRIVETRAIN_REAR_RIGHT_TALON));
                         

  private final DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
  private final Encoder m_driveEncoder = new Encoder(Constants.DRIVETRAIN_DRIVE_ENCODER_A, Constants.DRIVETRAIN_DRIVE_ENCODER_B);
  double EncoderValue = 0.0;

  public DriveTrain() {
  }

  public void arcadeDrive(double forward, double rotation) {
    // * forward=xSpeed - the robot's speed along the X axis [-1.0..1.0]. Forward is positive.
    // * rotation=zRotation - the robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
    m_drive.arcadeDrive(forward*-1, rotation);
  }

  public double getDriveEncoderCount() {
    return EncoderValue; //Timer.getMatchTime(); //m_driveEncoder.get();
	}

	public void resetDriveEncoderCount() {
    m_driveEncoder.reset();
    if (!RobotBase.isReal()) {
      EncoderValue = 0;
    }
  }
  
  public void incrementDriveEncoderCount() {
    EncoderValue++;
  }

  public void increaseDriveEncoderCount(double forward_amount) {
    EncoderValue = EncoderValue + forward_amount;
  }

	public double getDriveEncoderDistance() {
		return (getDriveEncoderCount() / (Constants.DRIVETRAIN_ENCODER_REVS_PER_FOOT)) * 12;
  }

  public void driveFeet(double feet) {
    this.resetDriveEncoderCount();
    
    // if Talon , can use control modes
    // TALON1.set(ControlMode.MotionMagic, -feet * Constants.DRIVETRAIN_ENCODER_REVS_PER_FOOT);
    // TALON2.follow(TALON1);
  }
}