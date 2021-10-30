// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private WPI_TalonSRX _rightTalon = new WPI_TalonSRX(Constants.rightWheelPort);
  private WPI_TalonSRX _leftTalon = new WPI_TalonSRX(Constants.leftWheelPort);
  private WPI_TalonSRX _armTalon = new WPI_TalonSRX(Constants.armPort);
  
  
  
  public Arm() {
      _leftTalon = new WPI_TalonSRX(Constants.leftWheelPort);
      _leftTalon.configFactoryDefault();
      _rightTalon = new WPI_TalonSRX(Constants.rightWheelPort);
      _rightTalon.configFactoryDefault();
      
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotContainer.getJoystick().getRawButton(1)){
      _leftTalon.set(ControlMode.PercentOutput, 0.4);
      _rightTalon.set(ControlMode.PercentOutput, 0.4);
    }
    else{
      _leftTalon.set(ControlMode.PercentOutput, 0.0);
      _rightTalon.set(ControlMode.PercentOutput, 0.0);
    }
  }
  public double getArmAngle(){
    return (_armTalon.getSelectedSensorPosition() / 4096.0) * 360;
  }
  public void setArmPower(double power){
    _armTalon.set(ControlMode.PercentOutput, power);
  }
}
