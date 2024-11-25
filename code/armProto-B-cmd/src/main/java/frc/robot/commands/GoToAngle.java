// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsys;
import static frc.robot.subsystems.ArmSubsys.*;

public class GoToAngle extends PIDCommand {
  double angleGo; // to display target angle v.i.

  /* Construct new GoToAngle cmd */
  public GoToAngle(ArmSubsys arm, double angle, double speed) {
    super(
        // k_ param come from subsystem import
        new PIDController(kP, kI, kD),
        // Close the loop by reading present angle
        arm::getAngle,
        // target angle
        angle,
        // Pipe output to arm subsys method
        output -> arm.armMotorSpark.set(output * speed),
        // Require the subsys instance
        arm); // end super

    // addRequirements(arm); not needed since super block added already
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    angleGo = angle;
    getController().setTolerance(0.1);

  } // end constructor

  // target angle will display on Smt Dash
  public void initialize() {
    Robot.angleGoal = (int) angleGo;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
} // end class
