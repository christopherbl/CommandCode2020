/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.PWMTalonFX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj.Encoder;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class PIDDriveTrain extends PIDSubsystem {

  //uncomment if you want to see output with simulation (can WPI_TalonSRX not simulation-capable)                             
  // private final SpeedControllerGroup m_pidleft =
  //     new SpeedControllerGroup(new PWMTalonFX(Constants.DRIVETRAIN_FRONT_LEFT_TALON_PID),
  //                             new PWMTalonFX(Constants.DRIVETRAIN_REAR_LEFT_TALON_PID));
                         
  // private final SpeedControllerGroup m_pidright =
  //     new SpeedControllerGroup(new PWMTalonFX(Constants.DRIVETRAIN_FRONT_RIGHT_TALON_PID),
  //                             new PWMTalonFX(Constants.DRIVETRAIN_REAR_RIGHT_TALON_PID));
  

  public final TalonSRX m_pidleft = new TalonSRX(Constants.DRIVETRAIN_FRONT_LEFT_TALON_PID);
  public final TalonSRX m_pidright = new TalonSRX(Constants.DRIVETRAIN_FRONT_RIGHT_TALON_PID);

  //private final DifferentialDrive m_piddrive = new DifferentialDrive(m_pidleft, m_pidright);
  private final Encoder m_driveEncoder = new Encoder(Constants.DRIVETRAIN_DRIVE_ENCODER_A_PID, Constants.DRIVETRAIN_DRIVE_ENCODER_B_PID);
  double pidEncoderValue;

  public final int allowableError = 100;

  
  public PIDDriveTrain() {
  super(
         // The PIDController used by the subsystem
         new PIDController(Constants.PID_DRIVE_P, Constants.PID_DRIVE_I, Constants.PID_DRIVE_D));   
         
         pidEncoderValue = 0.0;
  }
  
  public void setMotors(final double left, final double right) {
    m_pidleft.set(ControlMode.PercentOutput, left);
    m_pidright.set(ControlMode.PercentOutput, right);
  }

  @Override
  // changed following useOutput from protected to public so that PIDDriveCommand (different pkg) could reference it
  public void useOutput(final double output, final double setpoint) {
    setMotors(output,-output);
    this.increasePIDDriveEncoderCount(output);
  }

  @Override
  protected double getMeasurement() {
    return pidEncoderValue; //0;
  }

  public double getPIDDriveEncoderCount() {
    return pidEncoderValue; //Timer.getMatchTime(); //m_driveEncoder.get();
	}

	public void resetPIDDriveEncoderCount() {
    m_driveEncoder.reset();
    if (!RobotBase.isReal()) {
      pidEncoderValue = 0;
    }
  }

  public void increasePIDDriveEncoderCount(double forward_amount) {
    pidEncoderValue = pidEncoderValue + forward_amount;
  }

  public double getPIDDriveEncoderDistance() {
		return (getPIDDriveEncoderCount() / (Constants.DRIVETRAIN_ENCODER_REVS_PER_FOOT)) * 12;
  }
  
}
