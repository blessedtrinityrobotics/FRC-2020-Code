/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivebase;
import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Drivebase drivetrain = new Drivebase();

  //private final Drive drive = new Drive(drivetrain);

  // Starts Xbox Controllers
  public XboxController driverController = new XboxController(Constants.driveControllerPort);
  public XboxController operatorController = new XboxController(Constants.operatorControllerPort);

  // Starts Driver Buttons
  Button xButtonDriver             = new JoystickButton(driverController, Constants.xButton);
  Button aButtonDriver             = new JoystickButton(driverController, Constants.aButton);
  Button bButtonDriver             = new JoystickButton(driverController, Constants.bButton);
  Button yButtonDriver             = new JoystickButton(driverController, Constants.yButton);
  Button backButtonDriver          = new JoystickButton(driverController, Constants.backButton);
  Button startButtonDriver         = new JoystickButton(driverController, Constants.startButton);
  Button leftBumperButtonDriver    = new JoystickButton(driverController, Constants.leftBumperButton);
  Button rightBumperButtonDriver   = new JoystickButton(driverController, Constants.rightBumperButton);
  Button leftStickButtonDriver     = new JoystickButton(driverController, Constants.leftStickButton);
  Button rightStickButtonDriver    = new JoystickButton(driverController, Constants.rightStickButton);

  // Starts Operator Buttons
  Button xButtonOperator           = new JoystickButton(operatorController, Constants.xButton);
  Button aButtonOperator           = new JoystickButton(operatorController, Constants.aButton);
  Button bButtonOperator           = new JoystickButton(operatorController, Constants.bButton);
  Button yButtonOperator           = new JoystickButton(operatorController, Constants.yButton);
  Button backButtonOperator        = new JoystickButton(operatorController, Constants.backButton);
  Button startButtonOperator       = new JoystickButton(operatorController, Constants.startButton);
  Button leftBumperButtonOperator  = new JoystickButton(operatorController, Constants.leftBumperButton);
  Button rightBumperButtonOperator = new JoystickButton(operatorController, Constants.rightBumperButton);
  Button leftStickButtonOperator   = new JoystickButton(operatorController, Constants.leftStickButton);
  Button rightStickButtonOperator  = new JoystickButton(operatorController, Constants.rightStickButton);

  

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    drivetrain.setDefaultCommand( new Drive(drivetrain));                           
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    

    // ADD COMMANDS HERE

  }

  /**
   * Driver Axis
   * 
   * @param axis Axis number to return
   * @return Values of axis
   */
  public double getDriverRawAxis(int axis) {
    //System.out.println("Left Stick X: " + driverController.getRawAxis(axis));
    if( Math.abs(driverController.getRawAxis(axis)) < 0.01){
      return 0;
    } else {
      return driverController.getRawAxis(axis);
    } 
  }

  /**
   * Operator Axis
   * 
   * @param axis Axis number to return
   * @return Values of axis
   */
  public double getOperatorRawAxis(int axis) {
    return operatorController.getRawAxis(axis);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command generateRamseteCommand(Trajectory trajectory) {
    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory,
      drivetrain::getPose, 
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), 
      Constants.kDriveFF,
      Constants.kDriveKinematics, 
      drivetrain::getWheelSpeeds, 
      new PIDController(Constants.kPDriveVel, 0, 0), 
      new PIDController(Constants.kPDriveVel, 0, 0), 
      drivetrain::setDriveVolts,
      drivetrain
    );
    
  return ramseteCommand;
  }
  

  //public Command one(){
    // RamseteCommand ramseteCommand = new RamseteCommand(
    //   // get balls under trench trajectory,
    //   drivetrain::getPose, 
    //   new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta), 
    //   new SimpleMotorFeedforward(Constants.ksVolts, 
    //                              Constants.kvVoltSecondsPerMeter, 
    //                              Constants.kaVoltSecondsSquaredPerMeter),
    //   Constants.kDriveKinematics, 
    //   drivetrain::getWheelSpeeds, 
    //   new PIDController(Constants.kPDriveVel, 0, 0), 
    //   new PIDController(Constants.kPDriveVel, 0, 0), 
    //   drivetrain::setDriveVolts,
    //   drivetrain
    // );

    // Drive drive1 = new Drive(drivetrain);

    // return drive1.andThen(ramseteCommand);

    

    // drive forwrd 2 secs
    // shooter
    // turn 180
    // initiate ramsete command
    

      // turn 180

      // ramsete go back
      // shooter

      //stop


  

 // }

  
}
