// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SetArmAngle extends CommandBase {
  PIDController pid = new PIDController(Constants.pidPorts.pidP, Constants.pidPorts.pidI, Constants.pidPorts.pidD);
  private double angleArm;
  private ArmFeedforward feedForward = new ArmFeedforward(0.0, 0.0, 0.0);
  /** Creates a new SetArmAngle. */
  public SetArmAngle(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.getArm());
    angleArm = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(angleArm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.getArm().setArmPower(pid.calculate(RobotContainer.getArm().getArmAngle())+feedForward.calculate(RobotContainer.getArm().getArmAngle(), 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
