// armProto-B-cmd               Robot.j  cmd/subsys basic format

/* example for testing arm prototype, learning coding PID 
* position control of a single motor-controlled arm with various motor 
* controller hardware and feedback encoders
v. B --make PIDcmd subclass for button and Auto to call w/ param to set angle
and speed, make Sequ. group. Also added option for ProfilePIDcommand
*/

/* after deploying, enable teleOp, use L or R POV button to move arm to full
 reverse position, click L bumper button to zero encoder there, then manually
 control angle with POV's, or go to preset angles by holding buttons A,B,X,Y
*/

/* learning exercise for students: A. figure out what these lambda phrases 
* are doing and rewrite in clearer normal code (Trigger(method::reference), 
* InstantCommand()) -- using classes and methods not needing lambda operators.
* B. instead of space consuming new PIDCommand for each trigger action, make 
* one Command class that receives an angle param to turn to.
* C. make SmtDash interactive so you can tune PID param from there vs.
* redeploy when tuning k_ var's  -- add code to rP that reads present field
* value and changes the kP, kI,.. vars to SmtDash value if it's changed.
*/

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.GoToAngle;
import frc.robot.commands.GoToAngleProf;
import frc.robot.subsystems.ArmSubsys;
import static frc.robot.subsystems.ArmSubsys.*;

/**
 * The VM is configured to automatically run this class, and call the
 * functions corresponding to each mode, as described in TimedRobot doc.
 * -- in limited cmd/subsys format, no RC, all init done in roboInit.
 */
public class Robot extends TimedRobot {

  // instance joystick --assumes xbox-type gamepad plugged into port 0
  public final XboxController myStick = new XboxController(0);

  public final ArmSubsys myArmProto = new ArmSubsys();

  int angleGoal = 0;
  int rotaGoal = 0;

  @Override
  public void robotInit() {
    // declares, instances, configs this robot's specific components;
    // their functionality (methods) in subsys (when they exist).
    // these things done in RobotContainer() in more complex program

    // configuring button binding -- here, rI vs. rC for simplicity
    // define button triggers, used in rP and tP

    // resets encoder to show distance of 0 at current position
    Trigger leftBump = new Trigger(myStick::getLeftBumperPressed);

    Trigger buttonA = new Trigger(myStick::getAButton);

    // JoystickButton buttonB = new JoystickButton(myStick,
    // XboxController.Button.kB.value);
    Trigger buttonB = new Trigger(myStick::getBButton);
    Trigger buttonX = new Trigger(myStick::getXButton);
    Trigger buttonY = new Trigger(myStick::getYButton);

    Trigger armRevrs = new POVButton(myStick, 270); // left POV
    Trigger armForwd = new POVButton(myStick, 90); // rt POV

    // trigger work here (in rI) to move arm, but only when Enabled
    armForwd.onTrue(new InstantCommand(() -> myArmProto.armMotorSpark.set(0.35)));
    armForwd.onFalse(new InstantCommand(() -> myArmProto.armMotorSpark.set(0.0)));
    // both started movement onTrue, didn't stop until .onFalse added,
    armRevrs.onTrue(new InstantCommand(() -> myArmProto.armMotorSpark.set(-0.35)));
    armRevrs.onFalse(new InstantCommand(() -> myArmProto.armMotorSpark.set(0.0)));

    // method of encoder superclass; works here only when Enabled;
    leftBump.onTrue(new InstantCommand(() -> myArmProto.armEncoder.reset()));

    // press x 1 returns arm to home (full reverse) angle, slowly
    buttonA.onTrue(new PrintCommand("buttonA press"));
            // .onTrue(
        // new PIDCommand(new PIDController(kP, kI, kD),
        // // Close the loop by reading present angle
        // myArmProto::getAngle,
        // // target angle
        // setpointA,
        // // Pipe output to arm subsys method
        // output -> myArmProto.armMotorSpark.set(output * 0.1),
        // // Require the subsys instance
        // myArmProto))
    // .onFalse(new InstantCommand(() -> myArmProto.armMotorSpark.set(0.0)));
    // onFalse not needed for one-off action, so PIDcmd stays active, holding pos

    // call GoToAngle PIDcmd if held(whileTrue) vs. one Press (onTrue)
    buttonB.onTrue(new PrintCommand("buttonBpidCmd press"))
        .whileTrue(new GoToAngle(myArmProto, setpointB / 10, 0.15)) // 60deg
        .onFalse(new InstantCommand(() -> myArmProto.armMotorSpark.set(0.0)));

    // call GoToAngleProf (90 deg) on one press on buttonX  vs. press+hold .onWhile
    buttonX.onTrue(new PrintCommand("buttonXprof press"))
        .onTrue(new GoToAngleProf(myArmProto, setpointX / 10, 0.15)); // 90deg
        //.onFalse(new InstantCommand(() -> myArmProto.armMotorSpark.set(0.0)));

       // turn to 120deg on one press
    buttonY.onTrue(new PrintCommand("buttonYinlineCmd press"))
        .onTrue(new PIDCommand(new PIDController(kP, kI, kD),
            // Close the loop by reading present angle
            myArmProto::getRot,
            // Setpoint 120
            setpointY / 10, // can't .setTol for inline PIDcontrol?
            // Pipe output to arm subsys motor's method
            output -> myArmProto.armMotorSpark.set(output * 0.15),
            // Require the subsys
            myArmProto));
       // .onFalse(new InstantCommand(() -> myArmProto.armMotorSpark.set(0.0)));

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);

    // init SmtDsh field for desired angle goal and present angle
    SmartDashboard.putNumber("rotaGoal", 0);
    // field shows present encoder value
    SmartDashboard.putNumber("rotaCnt", 0.00);
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
      angleGoal = 0;
    if (myStick.getBButtonPressed())
      angleGoal = 60;
    if (myStick.getXButtonPressed())
      angleGoal = 90;
    if (myStick.getYButtonPressed())
      angleGoal = 120;

    CommandScheduler.getInstance().run();

    SmartDashboard.putNumber("rotaGoal", angleGoal);
    // display current arm angle
    SmartDashboard.putNumber("rotaCnt",
        myArmProto.getRot());
    // class method will return Math.round(absolArmEncod.getDistance()
    // myArmProto.getAngle();
  } // end robotPeriodic

  // autoInit runs the autonomous command set here or usually RC
  @Override
  public void autonomousInit() {

  } // end autoInit

  /** This function is called periodically during autonomous if needed */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This confirms that the autonomous code has stopped,. If you want
    // auto cmd to continue until interrupted, remove this line.
    // if (m_autonomousCommand != null) {
    // m_autonomousCommand.cancel();
    // }
    // m_robotContainer.m_drivetrain.resetEncoders();
    // m_robotContainer.m_drivetrain.resetGyro();
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
