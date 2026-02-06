package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.TurretSubsystemConstants;
import frc.robot.Constants.TurretSubsystemConstants.TurretSubSystemSetpoints;


/* NOTE: this Subsystem uses exact same Config and Constants of the original TurretSubsystem!!!! */
public class DynamicTurretSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum DynamicTurretSetpoints {
    kBase,
    kMoveRightSetpoint,
    kMoveLeftSetpoint,
    kPointAtTargetSetpoint,
  }

  // Initialize DynamicTurret SPARK. We will use MAXMotion position control for the DynamicTurret, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkFlex dynamicturretMotor =
      new SparkFlex(TurretSubsystemConstants.kTurretMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController dynamicturretController = dynamicturretMotor.getClosedLoopController();
  private RelativeEncoder dynamicturretEncoder = dynamicturretMotor.getEncoder();

  // Member variables for subsystem state management
  private double DynamicTurretCurrentTarget = TurretSubSystemSetpoints.kBase; // may have to start at 0

  // encoder setting determined from the target position for the turret - some angle calculations
  private double kPointAtTargetSetpointValue;

  // gear ratio
  private static final double GEAR_RATIO = 70.;
  
  public DynamicTurretSubsystem() {
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
    dynamicturretMotor.configure(
        Configs.TurretSubsystemConfiguration.turretConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


    // Zero DynamicTurret and elevator encoders on initialization
    dynamicturretEncoder.setPosition(0);

  }

  /**
   * Drive the DynamicTurret and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  public void moveToSetpoint() {
    dynamicturretController.setReference(DynamicTurretCurrentTarget, ControlType.kMAXMotionPositionControl);

  }

  /** Set DynamicTurret motor power in the range of [-1, 1]. - TEST Purpose: step through */
  private void setDynamicTurretPower(double power) {
    dynamicturretMotor.set(power);
  }

  /**
   * Command to set the subsystem setpoint. This will set the DynamicTurret and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(DynamicTurretSetpoints setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kMoveRightSetpoint:
              DynamicTurretCurrentTarget = TurretSubSystemSetpoints.kMoveRightSetpoint;
              break;
            case kMoveLeftSetpoint:
              DynamicTurretCurrentTarget = TurretSubSystemSetpoints.kMoveLeftSetpoint;
              break;
            case kBase:
              DynamicTurretCurrentTarget = TurretSubSystemSetpoints.kBase;
              break;
            case kPointAtTargetSetpoint:
              DynamicTurretCurrentTarget = kPointAtTargetSetpointValue;
              break;

          }
        });
  }


  /**
   * Command to run the dynamicturret motor. 
   * Intended to step through to adjust proper setpoints for elevator heights
   * When the command is interrupted, e.g. the button is released, the motor will stop.
   */
  public Command runDynamicTurretRightCommand() {
    return this.startEnd(
        () -> this.setDynamicTurretPower(TurretSubsystemConstants.TurretSetpointTestSpeed), 
        () -> this.setDynamicTurretPower(0.0));
  }

  public Command runDynamicTurretLeftCommand() {
    return this.startEnd(
        () -> this.setDynamicTurretPower((-1) * TurretSubsystemConstants.TurretSetpointTestSpeed), 
        () -> this.setDynamicTurretPower(0.0));
  }


  @Override
  public void periodic() {
    moveToSetpoint();

    // Display subsystem values
    SmartDashboard.putNumber("DynamicTurret/Target Position", DynamicTurretCurrentTarget);
    SmartDashboard.putNumber("DynamicTurret/Actual Position", dynamicturretEncoder.getPosition());
    SmartDashboard.putNumber("DynamicTurret/Actual Angle", dynamicturretEncoder.getPosition() * (360. / GEAR_RATIO));
    
  }

  @Override
  public void simulationPeriodic() {
  }

  /* have the turret encoder position set through some angle calculations */
  public void setPointAtTargetSetpointValue(double value){
    // may be good enough to pass the angle to move and scale using the fixed setpoint values
    // for example, determine encoder values for 30deg, 45deg, 60deg, 90deg etc and interpolate?
    kPointAtTargetSetpointValue = value;

    // assign this value as the next setpoint such that when moveToSetpoint() is called, it will move there
    DynamicTurretCurrentTarget = kPointAtTargetSetpointValue;
  }

}