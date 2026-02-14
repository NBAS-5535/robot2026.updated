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
import frc.robot.Constants.GenericSubsystemConstants;
import frc.robot.Constants.GenericSubsystemConstants.GenericSubSystemSetpoints;
import edu.wpi.first.math.MathUtil;


/* NOTE: this Subsystem uses exact same Config and Constants of the original GenericSubsystem!!!! */
public class GenericSubsystem extends SubsystemBase {
  /** Subsystem-wide setpoints */
  public enum GenericSetpoints {
    kBase,
    kMoveRightSetpoint,
    kMoveLeftSetpoint,
    kPointAtTargetSetpoint,
  }

  // Initialize Generic SPARK. We will use MAXMotion position control for the Generic, so we also need to
  // initialize the closed loop controller and encoder.
  private SparkFlex genericMotor =
      new SparkFlex(GenericSubsystemConstants.kGenericMotorCanId, MotorType.kBrushless);
  private SparkClosedLoopController genericController = genericMotor.getClosedLoopController();
  private RelativeEncoder genericEncoder = genericMotor.getEncoder();

  // need swerve info
  private CommandSwerveDrivetrain mDrivetrain;
  // Member variables for subsystem state management
  private double GenericCurrentTarget = GenericSubSystemSetpoints.kBase; // may have to start at 0


  public GenericSubsystem(CommandSwerveDrivetrain drivetrain) {
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
    genericMotor.configure(
        Configs.GenericSubsystemConfiguration.genericConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    genericMotor.setInverted(true);
    // Zero Generic encoder on initialization
    genericEncoder.setPosition(0);

    // link to Swerve
    this.mDrivetrain = drivetrain;

  }

  /**
   * Drive the Generic and elevator motors to their respective setpoints. This will use MAXMotion
   * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
   * setpoints.
   */
  public void moveToSetpoint() {
    genericController.setReference(GenericCurrentTarget, ControlType.kMAXMotionPositionControl);

  }

  /** Set Generic motor power in the range of [-1, 1]. - TEST Purpose: step through */
  public void setGenericPower(double power) {
    genericMotor.set(power);
  }

    public double getPosition() {
       return genericMotor.getEncoder().getPosition(); 
      }
  /**
   * Command to set the subsystem setpoint. This will set the Generic and elevator to their predefined
   * positions for the given setpoint.
   */
  public Command setSetpointCommand(GenericSetpoints setpoint) {
    return this.runOnce(
        () -> {
          switch (setpoint) {
            case kMoveRightSetpoint:
              GenericCurrentTarget = GenericSubSystemSetpoints.kMoveRightSetpoint;
              break;
            case kMoveLeftSetpoint:
              GenericCurrentTarget = GenericSubSystemSetpoints.kMoveLeftSetpoint;
              break;
            case kBase:
              GenericCurrentTarget = GenericSubSystemSetpoints.kBase;
              break;
            case kPointAtTargetSetpoint:
              //GenericCurrentTarget = kPointAtTargetSetpointValue;
              break;

          }
        });
  }


  /**
   * Command to run the generic motor. 
   * Intended to step through to adjust proper setpoints for elevator heights
   * When the command is interrupted, e.g. the button is released, the motor will stop.
   */
  /* MUST COMMENT OUT  moveToSetpoint() in execute() to make this work!!! */
  public Command runGenericRightCommand() {
    return this.startEnd(
        () -> this.setGenericPower(GenericSubsystemConstants.GenericSetpointTestSpeed), 
        () -> this.setGenericPower(0.0));
  }

  public Command runGenericLeftCommand() {
    return this.startEnd(
        () -> this.setGenericPower((-1) * GenericSubsystemConstants.GenericSetpointTestSpeed), 
        () -> this.setGenericPower(0.0));

  }

  @Override
  public void periodic() {
    moveToSetpoint();
    // Display subsystem values
    SmartDashboard.putNumber("Generic/Target Position", GenericCurrentTarget);
    SmartDashboard.putNumber("Generic/CurrentEncoderPosition", genericEncoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
  }

}