// teamBFlex-C                       ArmFlexSubsys.j

/* subsystem for arm prototype control */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Timer;

//import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

public class ArmFlexSubsys extends SubsystemBase {

  // Vortex + Flex on CAN bus
  // cannot find any API for this class, no methods, etc
  public final CANSparkFlex armMotorFlex = new CANSparkFlex(10, MotorType.kBrushless);

// to get data from motor's built-in; why don't they use SparkRE?
  public final RelativeEncoder flexQuadEncoder;

  // if reading ext. Quad encoder on arm, hardwire to RIO DIO port 2,3
  // public final Encoder armEncoder = new Encoder(2, 3, true);

  // for REV thru bore 11-1271 using abs. mode, full 360° rot = 1024 count
  // remote absol. encoder on arm axis, wired to this RIO port
  public final DutyCycleEncoder absolArmEncod = new DutyCycleEncoder(0);

  // smoothe angular rate from motor quad encoder
  LinearFilter rateFilter = LinearFilter.movingAverage(10);
  private final Timer timer = new Timer();
  // private double lastPosition = 0; // use for absolEncod rate
  // private double lastTime = 0;
  private double indxStart = 0; // time when index() method begins
  private double rateNow = 0; // periodic update angular change/sec.
  private boolean indxMode = false; // indexing active or not

  // angle control setpoints for button triggers
  // v. B has new PIDcommand class to set angle and speed param
  public static double setpointA = 0; // ? reach actual 0
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
  public ArmFlexSubsys() {

    // invert / or not, so that positive V (to red wire) results in
    // arm moving "forward'. Normal polarity goes fwd on prototype but
    // absolute encoder reads 'fwd' as negative, so for RIO control I need
    // to invert. I.e. fwd for Spark limit sw. is reverse in code for
    // RIO & gamepad when this code is in control
    armMotorFlex.setInverted(true);

    // to get motor's built-in relative quad. encoder data
    flexQuadEncoder = armMotorFlex.getEncoder();
// vortex 7168 tick/rota
    flexQuadEncoder.setPositionConversionFactor(Math.PI * 2 / 7168);

// per SparkBase should be SparkClosedLoopController, but that doesn't work
// for CANSparkFlex, so must stay with WPIlib PIDcontroller in cmd for now
  //  SparkPIDController  sparkFlexPid = armMotorFlex.getPIDController();

    // set 'distance' (angle°) conversion factor for ext. absol. encod.
    absolArmEncod.setDistancePerRotation(360.0);

    // to read absol encoder angle in rad.
    // absolArmEncod.setDistancePerRotation(2 * Math.PI);

    timer.start();
    // lastTime = timer.get();

  } // end constructor

  // use new values from SmtDash
public void setP(double nuP){ kP = nuP;}
public void setI(double nuI){ kI = nuI;}
public void setD(double nuD){ kD = nuD;}

  // read rotation angle from Thru-Bore encoder
  public double getAngle() {  // in deg. from external encoder
    return Math.round(absolArmEncod.getDistance());
  }  // rate near zero used only to index absol. encoder at present
  public double getRate() { // edited to use more stable quad encoder value
    double rate = flexQuadEncoder.getVelocity();
    return rateFilter.calculate(rate);
  }

  // indexing method moves arm to full rev. (0 angle) on button press
  public void index() {
    indxMode = true;
    if (timer.get() == 0) { // it hasn't started yet
      timer.start();
    }
    indxStart = timer.get();
  
    System.out.println("end index() block");
  }

  @Override
  public void periodic() {
    // called once per scheduler run; displays Rate, runs indexing OK
    rateNow = getRate();  // from motor's relative quad encoder
    double indxDelay = timer.get() - indxStart; // sec. since index() started

    if (indxMode) {  //param here may need tuning
      this.armMotorFlex.set(-0.2);
      // Timer.delay(0.1); // even this causes Cmd Sccheduler Loop Overrun
      if (Math.abs(rateNow) <= 0.5 && indxDelay >= 2.0) {
        this.armMotorFlex.set(0.0);
        // Reset the encoder
        absolArmEncod.reset();
        indxMode = false;
        System.out.println("index() moved arm, set to 0 deg.");
      } else
        this.armMotorFlex.set(-0.2); // run slow rev. until stopped by limit sw.
    } // end if
    SmartDashboard.putNumber("angulaRate", rateNow);
    SmartDashboard.putNumber("indexDelay", indxDelay);
  } // end periodic

} // end class
