package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.TurretSubsystemConstants;
import frc.robot.Constants.TurretSubsystemConstants.TurretSubSystemSetpoints;
import edu.wpi.first.math.MathUtil;


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

  // need swerve info
  private CommandSwerveDrivetrain mDrivetrain;
  // Member variables for subsystem state management
  private double DynamicTurretCurrentTarget = TurretSubSystemSetpoints.kBase; // may have to start at 0

  // encoder setting determined from the target position for the turret - some angle calculations
  private double kPointAtTargetSetpointValue;

  // gear ratio
  private static final double GEAR_RATIO = 70.;
  // CPR for NOE/NEO 550
  private static final double CPR = 42.0;

  // Fuel Bucket Positions
  private static final double targetX = 11.7; //Blue=4.55;
  private static final double targetY = 3.97;

  private boolean trackingtarget = false;

  public DynamicTurretSubsystem(CommandSwerveDrivetrain drivetrain) {
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

    dynamicturretMotor.setInverted(true);
    // Zero DynamicTurret encoder on initialization
    dynamicturretEncoder.setPosition(0);

    // link to Swerve
    this.mDrivetrain = drivetrain;

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
  public void setDynamicTurretPower(double power) {
    dynamicturretMotor.set(power);
  }

    public double getPosition() {
       return dynamicturretMotor.getEncoder().getPosition(); 
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
              trackingtarget = false;
              DynamicTurretCurrentTarget = TurretSubSystemSetpoints.kMoveLeftSetpoint;
              break;
            case kBase:
              DynamicTurretCurrentTarget = TurretSubSystemSetpoints.kBase;
              break;
            case kPointAtTargetSetpoint:
              //DynamicTurretCurrentTarget = kPointAtTargetSetpointValue;
              trackingtarget = true;
              DynamicTurretCurrentTarget = setPointAtTargetSetpointValue();
              //SmartDashboard.putNumber("DynamicTurret/setpointcommandvalue", setPointAtTargetSetpointValue());
              break;

          }
        });
  }


  /**
   * Command to run the dynamicturret motor. 
   * Intended to step through to adjust proper setpoints for elevator heights
   * When the command is interrupted, e.g. the button is released, the motor will stop.
   */
  /* MUST COMMENT OUT  moveToSetpoint() in execute() to make this work!!! */
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
    if(trackingtarget){
      DynamicTurretCurrentTarget = setPointAtTargetSetpointValue();
    }
    moveToSetpoint();
    // Display subsystem values
    SmartDashboard.putNumber("DynamicTurret/Target Position", DynamicTurretCurrentTarget);
    SmartDashboard.putNumber("DynamicTurret/CurrentEncoderPosition", dynamicturretEncoder.getPosition());
    SmartDashboard.putNumber("DynamicTurret/CurrentEncoder-Adj.Angle", dynamicturretEncoder.getPosition() * (360. / GEAR_RATIO));
    //SmartDashboard.putNumber("DynamicTurret/Actual CPR-adjusted Angle", dynamicturretEncoder.getPosition() / encoderConversionFactor());
  }

  @Override
  public void simulationPeriodic() {
  }

  /* have the turret encoder position set through some angle calculations */
  public double setPointAtTargetSetpointValue(){
    // may be good enough to pass the angle to move and scale using the fixed setpoint values
    // for example, determine encoder values for 30deg, 45deg, 60deg, 90deg etc and interpolate?
    Pose2d robotPose = mDrivetrain.getCurrentPose();
    
    //double rotationangle = robotPose.getRotation().getDegrees();
    double dx = targetX - robotPose.getX();
    double dy = targetY - robotPose.getY();
    double targetFieldAngle = Math.atan2(dy, dx);
    
    
    //MyMathequation 
    
    double robotheading = robotPose.getRotation().getRadians();
    double turretangle = targetFieldAngle - robotheading;
    // Normalize to [-pi, pi] 
    turretangle = MathUtil.angleModulus(targetFieldAngle - robotheading);
    double turretRotations = (turretangle / (2 * Math.PI)) * GEAR_RATIO;
    SmartDashboard.putNumber("DynamicTurret/turretrotations", turretRotations);
    //SmartDashboard.putNumber("DynamicTurret/Targetx" , targetX);
    //SmartDashboard.putNumber("DynamicTurret/Targety" , targetY);
    //SmartDashboard.putNumber("DynamicTurret/RobotPoseX" , robotPose.getX());
    //SmartDashboard.putNumber("DynamicTurret/RobotPoseY" , robotPose.getY());
    SmartDashboard.putNumber("DynamicTurret/targetFieldAngle (rad)", targetFieldAngle);
    double angleInDegrees = targetFieldAngle * 180. / Math.PI;
    SmartDashboard.putNumber("DynamicTurret/targetFieldAngle (deg)", angleInDegrees);
    double compAngleInDegrees = angleInDegrees + 180.;
    SmartDashboard.putNumber("DynamicTurret/targetFieldAngleAdj (deg)", compAngleInDegrees);
    //double target = Math.atan2(targetX - dx , targetY - dy);
    //SmartDashboard.putNumber("DynamicTurret/targetFieldAngle (enc)", targetFieldAngle * 180. / Math.PI * encoderConversionFactor());
    //SmartDashboard.putNumber("DynamicTurret/targetangle(motorenc)", targetFieldAngle * encoderConversionFactor());
    // assign this value as the next setpoint such that when moveToSetpoint() is called, it will move there
    //targetFieldAngle = targetFieldAngle * 180. / Math.PI * encoderConversionFactor();
    //targetFieldAngle = (180. + compAngleInDegrees) / 360. * GEAR_RATIO;
    //targetFieldAngle = 90. - compAngleInDegrees;
    SmartDashboard.putNumber("DynamicTurret/setPointAngle (deg)", targetFieldAngle);
    // limit to -180 to 180 degrees
    //targetFieldAngle = MathUtil.inputModulus(targetFieldAngle,-180,180);
    //targetFieldAngle = Rotation2d.fromDegrees(targetFieldAngle).getDegrees();
    return turretRotations;
    //return targetFieldAngle;
  }

  public double encoderConversionFactor() {
    // conversion factor to convert from angle to encoder position
    // this is determined by the gear ratio and the encoder counts per revolution
    // for example, if the gear ratio is 70:1 and the encoder (NEO/NEO550) has 42 counts per revolution, 
    // then the conversion factor is 70 * 42 = 2940
    return GEAR_RATIO * CPR / 360.0; // encoder counts per degree of turret rotation
  }
}