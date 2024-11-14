// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

import frc.robot.subsystems.ArmSubsys;
import static frc.robot.subsystems.ArmSubsys.*;

public class GoToAngleProf extends ProfiledPIDCommand {
  /** Construct new GoToAngleProf */
  public GoToAngleProf(ArmSubsys arm, double angle, double speed) {
    super(
        new ProfiledPIDController(
            // PID param
            kP, kI, kD,
            // motion profile constraints
            new TrapezoidProfile.Constraints(2.0, 1.0)),
        // This should return the measurement
        arm::getRot,
        // This should return the goal (can also be a constant)
        angle,
        // Use the output (and setpoint, if desired) here
        (output, setpoint) -> arm.armMotorSpark.set(output * speed),
        // addRequirements()
        arm); //end super
    // Configure additional PID options by calling `getController`
    getController().setTolerance(1.5); // not being used?
  } // end constructor

  // Returns true when the command should end. Does not end now when at target.
  @Override
  public boolean isFinished() {
    return false;
  }
}
