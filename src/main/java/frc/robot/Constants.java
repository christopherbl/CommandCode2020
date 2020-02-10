/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static final int OI_DRIVER_CONTROLLER = 0;
	
	public static final int OI_DRIVER_CONTROLLER_X_AXIS = 1;
	public static final int OI_DRIVER_CONTROLLER_Y_AXIS = 0;

	public static final int READ_ENCODER_BUTTON = 1;   //xbox button A using button 1 for short distance as only button 1 and 2 work
	public static final int RESET_ENCODER_BUTTON = 2;  //xbox button B
	public static final int DRIVE_SHORT_DISTANCE_BUTTON = 3;   //xbox button X
	public static final int DRIVE_PID_DISTANCE_BUTTON = 4; //xbox button Y
	public static final int PID_DRIVE_DISTANCE_BUTTON = 6;  //xbox button right bumper (just beyond Y button on top of controller)

    public static final int DRIVETRAIN_FRONT_LEFT_TALON = 0;
	public static final int DRIVETRAIN_REAR_LEFT_TALON = 1;
	public static final int DRIVETRAIN_FRONT_RIGHT_TALON = 2;
	public static final int DRIVETRAIN_REAR_RIGHT_TALON = 3;

	public static final int DRIVETRAIN_FRONT_LEFT_TALON_PID = 4;
	public static final int DRIVETRAIN_REAR_LEFT_TALON_PID = 5;
	public static final int DRIVETRAIN_FRONT_RIGHT_TALON_PID = 6;
	public static final int DRIVETRAIN_REAR_RIGHT_TALON_PID = 7;

	public static final int DRIVETRAIN_DRIVE_ENCODER_A = 1;
	public static final int DRIVETRAIN_DRIVE_ENCODER_B = 2;

	public static final int DRIVETRAIN_DRIVE_ENCODER_A_PID = 3;
	public static final int DRIVETRAIN_DRIVE_ENCODER_B_PID = 4;

	public static final int DRIVETRAIN_ENCODER_REVS_PER_FOOT = 12;

	public static final double SHORT_DISTANCE_TO_DRIVE_IN_FEET = 350.0;

	public static final double LONG_DISTANCE_TO_DRIVE_IN_FEET = 8000.0;

	public static final double SETPOINT_DRIVE_DISTANCE_PID = 125.0;

	public static final double PID_DRIVE_P = 1.0;
	public static final double PID_DRIVE_I = 0.0;
	public static final double PID_DRIVE_D = 0.0;
}
