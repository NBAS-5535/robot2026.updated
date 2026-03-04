package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodSubsystemConstants;
import frc.robot.Constants.HoodSubsystemConstants.HoodSubSystemSetpoints;
import frc.robot.Constants.HoodSubsystemConstants;
import edu.wpi.first.math.MathUtil;


/* NOTE: this Subsystem uses exact same Config and Constants of the original HoodSubsystem!!!! */
public class HoodSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum HoodSetpoints {
    kBase,
    kMoveRightSetpoint,
    kMoveLeftSetpoint,
    kPointAtTargetSetpoint,
  }

  // Initialize Hood SPARK. We will use MAXMotion position control for the Hood, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkFlex hoodMotor = new SparkFlex(HoodSubsystemConstants.kHoodMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController hoodController = hoodMotor.getClosedLoopController();
  private RelativeEncoder hoodEncoder = hoodMotor.getEncoder();

  // need swerve info
  // Member variables for subsystem state management
  private double HoodCurrentTarget = HoodSubSystemSetpoints.kBase; // may have to start at 0

  private double m_motorPower = 0.0;

   /** Constructor */
  public HoodSubsystem() {
    /*
     * Apply the appropriate configurations to the SPARKs.
     *
     * kResetSafeParameters is used to get the SPARK to a known state. This
     * is useful in case the SPARK is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    hoodMotor.configure(
        Configs.HoodSubsystemConfiguration.hoodConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    hoodMotor.setInverted(true);
    // Zero Hood encoder on initialization
    hoodEncoder.setPosition(0);

  }

  /**
   * Drive the Hood and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  public void moveToSetpoint() {
    hoodController.setReference(HoodCurrentTarget, ControlType.kMAXMotionPositionControl);

  }

  /** Set Hood motor power in the range of [-1, 1]. - TEST Purpose: step through */
  public void setHoodPower(double power) {
    hoodMotor.set(power);
  }

    public double getPosition() {
       return hoodMotor.getEncoder().getPosition(); 
      }
  /**
   * Command to set the subsystem setpoint. This will set the Hood and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(HoodSetpoints setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kMoveRightSetpoint:
              HoodCurrentTarget = HoodSubSystemSetpoints.kMoveRightSetpoint;
              break;
            case kMoveLeftSetpoint:
              HoodCurrentTarget = HoodSubSystemSetpoints.kMoveLeftSetpoint;
              break;
            case kBase:
              HoodCurrentTarget = HoodSubSystemSetpoints.kBase;
              break;
            case kPointAtTargetSetpoint:
              //HoodCurrentTarget = kPointAtTargetSetpointValue;
              break;

          }
        });
  }

  @Override
  public void periodic() {
    //moveToSetpoint();
    // Display subsystem values
    SmartDashboard.putNumber("Hood/Target Position", HoodCurrentTarget);
    SmartDashboard.putNumber("Hood/CurrentEncoderPosition", hoodEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
  }

  /**
   * Command to run the intake motor. 
   * Intended to step through to adjust proper setpoints
   * When the command is interrupted, e.g. the button is released, the motor will stop.
   */
  public Command runHoodInCommand() {
    return this.startEnd(
        () -> this.resetHoodPower(HoodSubsystemConstants.HoodSetpointTestSpeed), 
        () -> this.resetHoodPower(0.0));
  }

  public Command runHoodOutCommand() {
    return this.startEnd(
        () -> this.resetHoodPower((-1) * HoodSubsystemConstants.HoodSetpointTestSpeed), 
        () -> this.resetHoodPower(0.0));
  }

     /** Set Hood motor power in the range of [-1, 1]. - TEST Purpose: step through */
  public void resetHoodPower(double power) {
    m_motorPower = power;
  }
}