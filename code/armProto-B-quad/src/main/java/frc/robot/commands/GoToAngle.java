// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.ArmSubsys;
import static frc.robot.subsystems.ArmSubsys.*;

public class GoToAngle extends PIDCommand {
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
        arm);

    // Use addRequirements() here to declare subsystem dependencies.
     addRequirements(arm);

// Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.5);  //not using this ??
   
  } // end constructor

  // Returns true when the command should end, i.e. at setpoint, unless it's
  // supposed to hold position, or in Sequence group (must end to advance seq.)
  // @Override
  public boolean isFinished() {
    return getController().atSetpoint();
 }
}
