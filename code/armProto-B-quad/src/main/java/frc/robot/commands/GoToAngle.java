// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.ArmSubsys;
import static frc.robot.subsystems.ArmSubsys.*;

public class GoToAngle extends PIDCommand {
  double angleGo;

  /* Construct new GoToAngle cmd */
  public GoToAngle(ArmSubsys arm, double angle, double speed) {
    super(
        // k_ param come from subsystem import
        new PIDController(kP, kI, kD),
        // Close loop by reading present rota. count
        arm::getRot,
        // target rota
        angle,
        // Pipe output to arm subsys method
        output -> arm.armMotorSpark.set(output * speed),
        // Require the subsys instance
        arm); // end super

    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(arm); ? if needed by cmd since controller got already

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.2);

    angleGo = angle;
  } // end constructor

  public void initialize() {
    SmartDashboard.putNumber("rotaGoal", angleGo);
  }

  // Returns true when the command should end, i.e. at setpoint, unless it's
  // supposed to hold position; NB each cmd must end for Seq Grp to advance
  // @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
