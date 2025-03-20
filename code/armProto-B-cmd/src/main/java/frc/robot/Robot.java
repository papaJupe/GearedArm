// armProto-B-cmd               Robot.j  cmd/subsys basic format

/* example for testing arm prototype, learning coding PID 
* position control of a single motor-controlled arm with various motor 
* controller hardware and feedback encoders
v. B add PIDcmd subclass for button and Auto to call w/ param for angle
and speed, use in auto Sequ.; add indexing method to auto-zero arm w/ button
*/

/* after deploying, enable teleOp, use L or R POV button to move arm to full
 reverse position, click L bumper button to zero encoder there, then manually
 control angle with POV's, or go to preset angles by holding buttons A,B,X,Y.
 Index arm to full rev. position 0 from teleOp enable + R bump button
*/

/* learning exercise for students: A. figure out what these lambda phrases 
* are doing and rewrite in clearer normal code (Trigger(method::reference), 
* InstantCommand()) -- using classes and methods not needing lambda operators.
* B. instead of space consuming new PIDCommand for each trigger action, make 
* one Command class that receives an angle param to turn to. Added indexing
* method to subsys, goes to 0 angle on start-up (assumes limit switch stops it)
* C. make SmtDash interactive so you can tune PID param from there vs.
* redeploy when tuning k_ var's  -- add code to rP that reads present field
* value and changes the kP, kI,.. vars to SmtDash value if it's changed.
*/

package frc.robot;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.GoToAngle;
import frc.robot.subsystems.ArmSubsys;
import static frc.robot.subsystems.ArmSubsys.*;

/**
 * The VM is configured to automatically run this class, and call the
 * functions corresponding to each mode, as described in TimedRobot doc.
 * code = limited cmd/subsys format, no RC, all init done in roboInit.
 */
public class Robot extends TimedRobot {

  // instance joystick --assumes xbox-like gamepad plugged into port 0
  public final XboxController myStick = new XboxController(0);

  public final ArmSubsys myArmProto = new ArmSubsys();

  public Command AutoCommand;

  public static int angleGoal = 0;

  @Override
  public void robotInit() {
    // declares, instances, configs this robot's specific components;
    // their functions (methods) defined in subsys.
    // -- these things done in RobotContainer() in more complex program

    // move arm to full rev., assumes limit switch stops @rm angle = 0
    // myArmProto.index(); <--- rI calls method but does nothing since
    // periodics not running yet

    // configure button binding -- here, rI vs. rC for simplicity
    // define button triggers, used in rP and tP

    // to reset encoder to show angle = 0 at current position
    Trigger leftBump = new Trigger(myStick::getLeftBumperPressed);
    // to index, i.e. move arm to full rev., 0 angle, w/ this button press
    Trigger rightBump = new Trigger(myStick::getRightBumperPressed);

    // button presses move arm to specific angle
    Trigger buttonA = new Trigger(myStick::getAButton);
    Trigger buttonB = new Trigger(myStick::getBButton);
    Trigger buttonX = new Trigger(myStick::getXButton);
    Trigger buttonY = new Trigger(myStick::getYButton);
    // POV press & hold moves manually
    Trigger armRevrs = new POVButton(myStick, 270); // left POV
    Trigger armForwd = new POVButton(myStick, 90); // rt POV

    // trigger work here (rI) to move arm, but only when Enabled
    armForwd.onTrue(new InstantCommand(() -> myArmProto.armMotorSpark.set(0.3)));
    armForwd.onFalse(new InstantCommand(() -> myArmProto.armMotorSpark.set(0.0)));
    // both started movement onTrue, didn't stop until .onFalse added,
    armRevrs.onTrue(new InstantCommand(() -> myArmProto.armMotorSpark.set(-0.3)));
    armRevrs.onFalse(new InstantCommand(() -> myArmProto.armMotorSpark.set(0.0)));

    // method of encoder superclass; works only when Enabled;
    leftBump.onTrue(new InstantCommand(() -> myArmProto.absolArmEncod.reset()));
    // move arm to full rev. position, reset absol encoder to 0
    rightBump.onTrue(new InstantCommand(() -> myArmProto.index()));

    // press x 1 returns arm to home (full reverse) angle, slowly
    buttonA.onTrue(
        new PIDCommand(new PIDController(kP, kI, kD),
            // Close the loop by reading present angle
            myArmProto::getAngle,
            // target angle
            setpointA,
            // Pipe output to arm subsys method
            output -> myArmProto.armMotorSpark.set(output * 0.1),
            // Require the subsys instance
            myArmProto))
        .onTrue(new PrintCommand("buttonA press"));
    // .onFalse(new InstantCommand(() -> myArmProto.armMotorSpark.set(0.0)));
    // onFalse not needed for one-off action, so PIDcmd stays active, holding pos

    // press & hold calls GoToAngle PIDcmd ; ? if .onFalse (cmd) needed
    buttonB
        .whileTrue(new GoToAngle(myArmProto, setpointB, 0.15)) // 60deg
        .onFalse(new InstantCommand(() -> myArmProto.armMotorSpark.set(0.0)))
        .onTrue(new PrintCommand("buttonB press"));

    // go to 90° on buttonX press + hold
    buttonX
        .whileTrue(new PIDCommand(new PIDController(kP, kI, kD),
            // Close the loop on present angle
            myArmProto::getAngle,
            // Setpoint 90
            setpointX,
            // Pipe output to armMotor method
            output -> myArmProto.armMotorSpark.set(output * 0.15),
            // Require the subsys
            myArmProto))
        .onFalse(new InstantCommand(() -> myArmProto.armMotorSpark.set(0.0)))
        .onTrue(new PrintCommand("buttonX press"));

    buttonY // press & hold --> 120 deg
        .whileTrue(new PIDCommand(new PIDController(kP, kI, kD),
            // Close the loop by reading present angle
            myArmProto::getAngle,
            // Setpoint 120
            setpointY, // ? possible to .setTol for this PIDcontrol?
            // Pipe output to arm subsys motor's method
            output -> myArmProto.armMotorSpark.set(output * 0.15),
            // Require the subsys
            myArmProto))
        .onFalse(new InstantCommand(() -> myArmProto.armMotorSpark.set(0.0)))
        .onTrue(new PrintCommand("buttonY press"));

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);

    // init SmtDsh fields for desired angle goal and present angle
    SmartDashboard.putNumber("angleGoal", 0);
    // field shows present encoder value
    SmartDashboard.putNumber("armAngle deg.", 0);
    // to put text on SmtDsh
    // SmartDashboard.putString("GTPcmd fin?", "???");

  } // end robotInit()

  // This function is called every robot packet, no matter the mode.
  // Does things you want run in all modes, like diagnostics.
  // This runs after the mode specific periodic functions, but before
  // LiveWindow and SmartDashboard integrated updating.
  @Override
  public void robotPeriodic() {
    // Calls the Scheduler <-- this is for polling buttons, adding
    // newly-scheduled commands, running now-scheduled commands,
    // removing finished commands, and running subsystem periodics.
    // Need CS.run for anything in the Cmd/Subsys framework to work

    // display selected angle (setpoint) on SmtDash
    if (myStick.getAButtonPressed())
      angleGoal = (int) setpointA;
    if (myStick.getBButtonPressed())
      angleGoal = (int) setpointB;
    if (myStick.getXButtonPressed())
      angleGoal = (int) setpointX;
    if (myStick.getYButtonPressed())
      angleGoal = (int) setpointY;

    CommandScheduler.getInstance().run();

    // comment out to show auto targets
    // SmartDashboard.putNumber("angleGoal", angleGoal);
    // display current arm angle
    SmartDashboard.putNumber("armAngle deg.", myArmProto.getAngle());
    // class method will return Math.round(absolArmEncod.getDistance()

  } // end robotPeriodic

  // autoInit runs the autonomous command set here, or usually in RC
  @Override
  public void autonomousInit() {
    AutoCommand = new SequentialCommandGroup(
        new GoToAngle(myArmProto, setpointB, 0.15)
            .andThen(new WaitCommand(2.0))
            .andThen(new GoToAngle(myArmProto, setpointX, 0.15))
            .andThen(new WaitCommand(2.0))
            .andThen(new GoToAngle(myArmProto, setpointY, 0.15))
            .andThen(new WaitCommand(2.0))
            .andThen(new GoToAngle(myArmProto, setpointA, 0.15))
            .andThen(new PrintCommand("auto GTAdone")));

    if (AutoCommand != null)
      AutoCommand.schedule();
  } // end autoInit

  /** This function is called periodically during autonomous if needed */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This confirms that the autonomous code has stopped. If you want
    // auto cmd to continue until interrupted, remove this line.
    // if (m_autonomousCommand != null) {
    // m_autonomousCommand.cancel();
    // }
    // m_robotContainer.m_drivetrain.resetEncoders();
    // m_robotContainer.m_drivetrain.resetGyro();

    // indexing method move arm to full rev. 0 angle on tele Enabl ?
    // myArmProto.index();

  } // end teleInit

  /* This function is called periodically when bot Enabled */
  @Override
  public void teleopPeriodic() {

  } // end telePeri

  // function is called once each time robot enters Disabled mode.
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
} // end class
