// armProto-B quad                             ArmSubsys.j

/* subsystem for arm prototype control project using CIM + relative encoder
 * req. by subclassed PIDcmd w/ angle & speed param., & profPIDcmd
 */
package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsys extends SubsystemBase {

  // single brushed motor, PWM control (port 0) of RoboRIO
  public final Spark armMotorSpark = new Spark(0);

  // if reading Quad encoder on arm, hardwire to RIO DIO port _a, _b
  // CIM quad encoder --> 1024 pulse/ rot.
  public final Encoder armEncoder = new Encoder(8, 9);

  // for REV thru bore 11-1271 using abs. mode, full 360° rot = 1024 count
  // remote absol. encoder on arm axis, wired to this RIO port
  // public final DutyCycleEncoder absolArmEncod = new DutyCycleEncoder(0);

  // angle control setpoints for button triggers / these/10 for rotat. target
  public static double setpointA = -3.0; // needed (-) offset to reach actual 0
  public static double setpointB = 60;
  public static double setpointX = 90;
  public static double setpointY = 120; // end limit ~180°

  // angle controller PID param
  public static double kP = 1.0;
  public static double kI = 1.5;
  public static double kD = 0.0;
  // not used yet
  // public static double kMaxOutput = 0.2;
  // public static double kMinOutput = 0.2;

  // Constructor
  public ArmSubsys() {

    // invert / or not, so that positive V (to red wire) results in
    // arm moving "forward'. Normal polarity goes fwd on prototype but
    // absolute encoder reads 'fwd' as negative, so for RIO control of arm
    // need to invert. I.e. fwd for Spark limit is reverse in code for
    // RIO & gamepad when this code is in control
    armMotorSpark.setInverted(false); // CIM needs false, CCW=fwd

    // set 'distance' (angle°) conversion factor for absol. encod.
    // absolArmEncod.setDistancePerRotation(360.0);
    //armEncoder.setDistancePerPulse(1 / 1024); get() failed w/ this
    armEncoder.setReverseDirection(true);

    // to read angle in rad.
    // absolArmEncod.setDistancePerRotation(2 * Math.PI);

    // how do I setTol for inline created PIDcontroller? A: Can't.
    // setpoint before it's counted as at the reference (setpt)
    // angleControl.setTolerance(2); set just distance
    // deg, deg/sec (dist , vel)

  } // end constructor

  // public double getAngle() {
  // return Math.round(absolArmEncod.getDistance());
  // }
  // to show rotation count
  public double getRot() {
    return (armEncoder.get() / 1024.00);
  }

  @Override
  public void periodic() {
    // This method is called once per scheduler run

  }
} // end class
