/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Drive extends CommandBase {

  
  //public RobotContainer robotContainer = new RobotContainer();
  //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public double rightTrigger;
  public double leftTrigger;
  public double leftStickX;
  public double rightStickY;
  private final Drivebase drivetrain;
  //private final RobotContainer robotContainer;


  public Drive(Drivebase subsystem) {
    drivetrain = subsystem;
    addRequirements(drivetrain);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //double leftStickY = robotContainer.getDriverRawAxis(Constants.leftStickY);
    rightTrigger = Robot.m_robotContainer.getDriverRawAxis(Constants.rightTriggerAxis);
    leftTrigger = Robot.m_robotContainer.getDriverRawAxis(Constants.leftTriggerAxis);
    leftStickX = Robot.m_robotContainer.getDriverRawAxis(Constants.leftStickX);
  /*
    // Tank Drive Options
      // Regular Tank Drive
      drivetrain.setLeftMotors(leftStickY);
      drivetrain.setRightMotors(rightStickY);
      // Exponential Tank Drive
      drivetrain.setLeftMotors(leftStickY * Math.abs(leftStickY));
      drivetrain.setRightMotors(rightStickY * Math.abs(rightStickY));
  */
    // Split Arcade Drive Options  
      // Regular Split Arcade
      //drivetrain.setLeftMotors(rightStickY + ((leftStickX * Math.abs(leftStickX)) * Constants.turningPower));
      //drivetrain.setRightMotors(rightStickY - ((leftStickX * Math.abs(leftStickX)) * Constants.turningPower));

       /*
      // Exponential Drive Split Arcade
      drivetrain.setLeftMotors((rightStickY * Math.abs(rightStickY)) + (leftStickX * Constants.turningPower));
      drivetrain.setRightMotors((rightStickY * Math.abs(rightStickY)) - (leftStickX * Constants.turningPower));
      // Exponential Drive and Turn Split Arcade 
      drivetrain.setLeftMotors((rightStickY * Math.abs(rightStickY)) + ((leftStickX * Math.abs(leftStickX)) * Constants.turningPower));
      Rdrivetrain.setRightMotors((rightStickY * Math.abs(rightStickY)) - ((leftStickX * Math.abs(leftStickX)) * Constants.turningPower));
    */
  
    // GTA Drive Options
      // Regular GTA Drive
      double triggerValue1 = rightTrigger - leftTrigger;
      double turnValue1 = -leftStickX * Constants.turningPower;
      drivetrain.setLeftMotors(-triggerValue1 + turnValue1);
      drivetrain.setRightMotors(-triggerValue1 - turnValue1);
      /*
      // Exponential Drive GTA Drive
      double triggerValue2 = (rightTrigger - leftTrigger) * Math.abs((rightTrigger - leftTrigger));
      double turnValue2 = leftStickX * Constants.turningPower;
      drivetrain.setLeftMotors(triggerValue2 + turnValue2);
      drivetrain.setRightMotors(triggerValue2 - turnValue2);
      // Exponential Drive and Turn GTA Drive
      double triggerValue3 = (rightTrigger - leftTrigger) * Math.abs((rightTrigger - leftTrigger));
      double turnValue3 = (leftStickX * Math.abs(leftStickX)) * Constants.turningPower;
      drivetrain.setLeftMotors(triggerValue3 + turnValue3);
      drivetrain.setRightMotors(triggerValue3 - turnValue3);
      */

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setLeftMotors(0);
    drivetrain.setRightMotors(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
