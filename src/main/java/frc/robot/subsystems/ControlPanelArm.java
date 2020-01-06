/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;



public class ControlPanelArm extends SubsystemBase {

  public VictorSPX controlPanelMotor = new VictorSPX(Constants.controlPanelMotorPort);
  public final I2C.Port i2cPort = I2C.Port.kOnboard;
  public final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  


  public ControlPanelArm() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * 
   * @return Best color guess
   */
  public Color getColors(){
    return colorSensor.getColor();
  }

  /**
   * 
   * @return Best color guess in string
   */
  public String stringColor(){
    return "" + getColors().toString();
  }

  /**
   * 
   * @return Red value of detected color
   */
  public double getRedColor(){
    return colorSensor.getColor().red;
  }

  /**
   * 
   * @return Blue value of detected color
   */
  public double getBlueColor(){
    return colorSensor.getColor().blue;
  }

  /**
   * 
   * @return Green value of detected color
   */
  public double getGreenColor(){
    return colorSensor.getColor().green;
  }

  
  
}
