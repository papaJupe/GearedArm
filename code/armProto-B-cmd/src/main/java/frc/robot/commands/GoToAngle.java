// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.ArmSubsys;
import static frc.robot.subsystems.ArmSubsys.*;
//import frc.robot.Robot.*;
//import frc.robot.Robot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToAngle extends PIDCommand {
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
        arm);

    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  } // end constructor

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
