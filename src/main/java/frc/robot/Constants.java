/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Motor Ports
    // Left Drive Train GB Motors
    public static final int  leftMasterMotorPort     = 5;
    public static final int  leftSlaveMotor1Port     = 8;
    // Right Drive Train GB Motors
    public static final int  rightMasterMotorPort    = 7;
    public static final int  rightSlaveMotor1Port    = 4;
    // Control Panel Motor
    public static final int controlPanelMotorPort    = 0;
  //End of Motor Ports

  // Pigeon IMU
    public static final int pigeonIMUPort            = 1;
  //End of Pigeon IMU

  // XboxController Ports
    public static final int driveControllerPort      = 0;
    public static final int operatorControllerPort   = 1;
  //End of XboxController Ports

  //Constants
    public static final int operatingVoltage          = 11;
    public static final int wheelDiameter             = 4;    // Wheel Diameter In Inches
    public static final double wheelDiameterMeters    = wheelDiameter/39.37;  // Wheel diameter in Meters
    public static final double gearRatio              = 9.97; // Gear Ratio
    public static final int CPR                       = 2048; // Falcon 500 encoder Counts per Revolution
    public static final int kMaxRPM                   = 6380; // Falcon 500 RPM
    public static final double turningPower           = 0.5;  // Turning Power for Drive (%)
    public static final double waitTime               = 1.0;  // Time to wait before grabing final gyro angle for vision approach
  //End of Constants

  //Axis
    public static final int leftStickX       = 0;  
    public static final int leftStickY       = 1;
    public static final int leftTriggerAxis  = 2;
    public static final int rightTriggerAxis = 3;
    public static final int rightStickX      = 4;
    public static final int rightStickY      = 5;
  //End of Axis

  //Buttons
    public static final int aButton           = 1;
    public static final int bButton           = 2;
    public static final int xButton           = 3;
    public static final int yButton           = 4;
    public static final int leftBumperButton  = 5;
    public static final int rightBumperButton = 6;
    public static final int backButton        = 7;
    public static final int startButton       = 8;
    public static final int leftStickButton   = 9;
    public static final int rightStickButton  = 10;
  //End of Buttons

  /**
	 * Set to zero to skip waiting for confirmation.
	 * Set to nonzero to wait and report to DS if action fails.
	 */
  public final static int kTimeoutMs = 30;
    
  //DriveTrain Motion Magic Constants
  public static final int kDriveTrainSensorVel                      = 1000;
  public static final int kDriveTrainAccel                          = kDriveTrainSensorVel/2;
  public static final int kDriveTrainVelocity                       = kDriveTrainSensorVel/2;
  public static final double ksVolts                                = 0.0;
  public static final double kvVoltSecondsPerMeter                  = 0.0;
  public static final double kaVoltSecondsSquaredPerMeter           = 0.0;
  public static final double kPDriveVel                             = 0.0;
  public static final double kTrackwidthMeters                      = 0.0;
  public static final double kMaxSpeedMetersPerSecond               = 0.0;
  public static final double kMaxAccelerationMetersPerSecondSquared = 0.0;
  public static final double kRamseteB                              = 2.0;
  public static final double kRamseteZeta                           = 0.7;
/*
  public static final DifferentialDriveKinematics kDriveKinematics  = 
        new DifferentialDriveKinematics(kTrackwidthMeters);
  
  public static final SimpleMotorFeedforward kDriveFF = 
        new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);

  public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = 
        new DifferentialDriveVoltageConstraint(kDriveFF, kDriveKinematics, operatingVoltage);

  public static final TrajectoryConfig defaultConfig = 
        new TrajectoryConfig(kMaxSpeedMetersPerSecond, 
                             kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(kDriveKinematics)
            .addConstraint(autoVoltageConstraint);
*/


  /**
   * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
     * Not all set of Gains are used in this project and may be removed as desired.
     * 
   * 	                                    	       kP   kI   kD   kF                          Iz   PeakOut */
  public final static Gains kGains_Drive = new Gains( 0.0, 0.0,  0.0, 0.0, 100,  0.50 );




  /** ---- Flat constants, you should not need to change these ---- */
  /* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
  public final static int REMOTE_0 = 0;
  public final static int REMOTE_1 = 1;
  /* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
  public final static int PID_PRIMARY = 0;
  public final static int PID_TURN    = 1;
  /* ---- Named slots, used to clarify code ---- */
  public final static int kSlot_Drive = 0;


}
