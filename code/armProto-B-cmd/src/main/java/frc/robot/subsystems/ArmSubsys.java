// armProto-B-cmd                             ArmSubsys.j

/* subsystem for arm prototype control project */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;

public class ArmSubsys extends SubsystemBase {

  // single brushed motor, PWM control (port) of RoboRIO
  public final Spark armMotorSpark = new Spark(0);

  // if reading Quad encoder on arm, hardwire to RIO DIO port 2,3
  public final Encoder armEncoder = new Encoder(2, 3, true);

  // for REV thru bore 11-1271 using abs. mode, full 360° rot = 1024 count
  // remote absol. encoder on arm axis, wired to this RIO port
  public final DutyCycleEncoder absolArmEncod = new DutyCycleEncoder(0);
  // smoothe angular rate
  private final LinearFilter rateFilter = LinearFilter.movingAverage(10);
  private final Timer timer = new Timer();
  // private double lastPosition = 0; // use for absolEncod rate
  // private double lastTime = 0;
  private double indxStart = 0; // time when index() method begins
  private double rateNow = 0; // periodic update angular change/sec.
  private boolean indxMode = false; // indexing active or not

  // angle control setpoints for button triggers
  // v. B has new PIDcommand class to set angle and speed param
  public static double setpointA = -2.0; // needed this (-) offset to reach actual 0
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
    // to invert. I.e. fwd for Spark limit sw. is reverse in code for
    // RIO & gamepad when this code is in control
    armMotorSpark.setInverted(true);

    // set 'distance' (angle°) conversion factor for absol. encod.
    absolArmEncod.setDistancePerRotation(360.0);

    // to read angle in rad.
    // absolArmEncod.setDistancePerRotation(2 * Math.PI);

    timer.start();
    // lastTime = timer.get();

  } // end constructor

  public double getAngle() {
    return Math.round(absolArmEncod.getDistance());
  }

  public double getRate() { // edited to use more stable quad encoder #
    double rate = armEncoder.getRate();
    return rateFilter.calculate(rate);
  }

  // public double getRate() { // edited method appears to work but jittery
  // double currentPosition = absolArmEncod.getDistance(); // more precise number
  // double currentTime = timer.get();
  // double rate = (currentPosition - lastPosition) / (currentTime - lastTime);
  // lastPosition = currentPosition;
  // lastTime = currentTime;
  // return rateFilter.calculate(rate);
  // }

  // indexing method moves arm to full rev., 0 angle on button Trigger
  public void index() {
    indxMode = true;
    // proposed isRunning() method was good conceptually but ...
    if (timer.get() == 0) { // isRunning() method !exist for Timer class
      timer.start();
    }
    indxStart = timer.get();
    // Set motor ops moved to periodic()
    System.out.println("end index() block");
  }

  @Override
  public void periodic() {
    // called once per scheduler run; displays Rate, runs indexing OK
    rateNow = getRate(); // updates lastPosition and lastTime (absolEncod rate)
    double indxDelay = timer.get() - indxStart; // sec. since index() started
    if (indxMode) {
      this.armMotorSpark.set(-0.2);
      // Timer.delay(0.1); // even this causes Cmd Sccheduler Loop Overrun
      if (Math.abs(rateNow) <= 0.5 && indxDelay >= 2.0) {
        this.armMotorSpark.set(0.0);
        // Reset the encoder
        absolArmEncod.reset();
        indxMode = false;
        System.out.println("index() moved arm, set to 0 deg.");
      } else
        this.armMotorSpark.set(-0.2); // run slow rev. until stopped by limit sw.
    } // end if
    SmartDashboard.putNumber("angulaRate", rateNow);
    SmartDashboard.putNumber("indexDelay", indxDelay);
  } // end periodic

} // end class
