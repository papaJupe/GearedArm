// armProto-B-cmd                             ArmSubsys.j

/* subsystem for arm prototype control project */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;

public class ArmSubsys extends SubsystemBase {

  // single brushed motor, PWM control (port) of RoboRIO
  public final Spark armMotorSpark = new Spark(0);

  // if reading Quad encoder on arm, hardwire to RIO DIO port 0,1
  // public final Encoder armEncoder = new Encoder(0,1);

  // for REV thru bore 11-1271 using abs. mode, full 360° rot = 1024 count
  // remote absol. encoder on arm axis, wired to this RIO port
  public final DutyCycleEncoder absolArmEncod = new DutyCycleEncoder(0);
  private final LinearFilter rateFilter = LinearFilter.movingAverage(5);
  private final Timer timer = new Timer();
  private double lastPosition = 0;
  private double lastTime = 0;

  // angle control setpoints for button triggers
  // I would prefer to have each button set a param, sending it to
  // a new PIDcommand using that angle param. Expect to need named PIDcmd class
  public static double setpointA = -3.0; // needed this (-) offset to reach actual 0
  public static double setpointB = 60;
  public static double setpointX = 90;
  public static double setpointY = 120; // end limit ~180°

  // angle controller PID param
  public static double kP = 0.35;
  public static double kI = 0.45;
  public static double kD = 0.0;
  // not used yet
  // public static double kMaxOutput = 0.2;
  // public static double kMinOutput = 0.2;

  // Constructor
  public ArmSubsys() {

    // invert / or not, so that positive V (to red wire) results in
    // arm moving "forward'. Normal polarity goes fwd on prototype but
    // absolute encoder reads 'fwd' as negative, so for RIO control I need
    // to invert. I.e. fwd for Spark limit is reverse in code for
    // RIO & gamepad when this code is in control
    armMotorSpark.setInverted(true);

    // set 'distance' (angle°) conversion factor for absol. encod.
    absolArmEncod.setDistancePerRotation(360.0);

    // to read angle in rad.
    // absolArmEncod.setDistancePerRotation(2 * Math.PI);

    timer.start();
    lastTime = timer.get();

  } // end constructor

  public double getAngle() {
    return Math.round(absolArmEncod.getDistance());
  }

  public double getRate() {
    double currentPosition = absolArmEncod.getDistance();
    double currentTime = timer.get();
    double rate = (currentPosition - lastPosition) / (currentTime - lastTime);
    lastPosition = currentPosition;
    lastTime = currentTime;
    return rateFilter.calculate(rate);
  }

  // indexing method moves arm to full rev. 0 angle
  public void index() {
    // Start the timer if it's not running
    if (timer.get() == 0) {
      timer.start();
    }

    // Set motor to slow reverse speed
    armMotorSpark.set(-0.05);

    // Wait until arm stops moving
    while (Math.abs(getRate()) > 0.5) {
      // Continues running periodic functions
      Timer.delay(0.02);
    }

    //  then Stop the motor
    armMotorSpark.set(0);

    // Reset the encoder
    absolArmEncod.reset();

    System.out.println("Indexing fin, Arm at 0 deg.");
  }  // end indexing

  @Override
  public void periodic() {
    // called once per scheduler run
    double rateNow = getRate(); // updates lastPosition and lastTime
    SmartDashboard.putNumber("angulaRate", rateNow);
  }
} // end class
