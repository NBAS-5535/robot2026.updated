package frc.robot;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CANRangeConstants;
import frc.robot.Constants.HoodSubsystemConstants;
import frc.robot.Constants.HopperSubsystemConstants;
import frc.robot.Constants.IntakeSubsystemConstants;
import frc.robot.Constants.TurretSubsystemConstants;

public class Configs {

 
    /* *****************
    * TurretSubsystem 
    */
   public static final class TurretSubsystemConfiguration {
     public static final SparkFlexConfig turretConfig = new SparkFlexConfig();
 
     static {
       // Configure basic setting of the turret motor
       turretConfig
         .smartCurrentLimit(TurretSubsystemConstants.kTurretCurrentLimit)
         .closedLoopRampRate(TurretSubsystemConstants.kTurretRampRate);
 
       /*
        * Configure the closed loop controller. We want to make sure we set the
        * feedback sensor as the primary encoder.
        */
       turretConfig
           .closedLoop
           .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
           // Set PID values for position control. We don't need to pass a closed
           // loop slot, as it will default to slot 0.
           .pid(TurretSubsystemConstants.kTurretKp, 
               TurretSubsystemConstants.kTurretKi, 
               TurretSubsystemConstants.kTurretKd)
           .outputRange(-1,1)
           .maxMotion
           // Set MAXMotion parameters for position control
           .maxVelocity(320000)//used to be 2000
           .maxAcceleration(1280000)//used to be 10000
           .allowedClosedLoopError(0.25);
 
       turretConfig.idleMode(IdleMode.kBrake);
     }
   }

   /* *****************
    * HoodSubsystem 
    */
   public static final class HoodSubsystemConfiguration {
     public static final SparkFlexConfig hoodConfig = new SparkFlexConfig();
 
     static {
       // Configure basic setting of the hood motor
       hoodConfig
         .smartCurrentLimit(HoodSubsystemConstants.kHoodCurrentLimit)
         .closedLoopRampRate(HoodSubsystemConstants.kHoodRampRate);
 
       /*
        * Configure the closed loop controller. We want to make sure we set the
        * feedback sensor as the primary encoder.
        */
       hoodConfig
           .closedLoop
           .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
           // Set PID values for position control. We don't need to pass a closed
           // loop slot, as it will default to slot 0.
           .pid(HoodSubsystemConstants.kHoodKp, 
               HoodSubsystemConstants.kHoodKi, 
               HoodSubsystemConstants.kHoodKd)
           .outputRange(-1,1)
           .maxMotion
           // Set MAXMotion parameters for position control
           .maxVelocity(320000)//used to be 2000
           .maxAcceleration(1280000)//used to be 10000
           .allowedClosedLoopError(0.25);
 
       hoodConfig.idleMode(IdleMode.kBrake);
     }
   }

   public static final class CANRangeConfiguration {

    public static final CANrangeConfiguration proximitySensorConfig = new CANrangeConfiguration();

    static {
      // If CANrange has a signal strength of at least 2000, it is a valid measurement.
      proximitySensorConfig.ProximityParams.MinSignalStrengthForValidMeasurement = CANRangeConstants.kMinSignalStrengthForValidMeasurement;
      // If CANrange detects an object within 0.1 meters, it will trigger the "isDetected" signal.
      proximitySensorConfig.ProximityParams.ProximityThreshold = CANRangeConstants.kProximityThreshold; 
      // Make the CANrange update as fast as possible at 100 Hz. This requires short-range mode.
      proximitySensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz; 
    }
   }   

  /* *****************
   * IntakeSubsystem 
   */
  public static final class IntakeSubsystem {
    public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();

    static {
      // Configure basic setting of the intake motor
      intakeConfig
        .smartCurrentLimit(IntakeSubsystemConstants.kIntakeCurrentLimit)
        .closedLoopRampRate(IntakeSubsystemConstants.kIntakeRampRate);

      intakeConfig.idleMode(IdleMode.kBrake);
    }
  }

    /* *****************
   * HopperSubsystem 
   */
  public static final class HopperSubsystem {
    public static final SparkFlexConfig hopperConfig = new SparkFlexConfig();

    static {
      // Configure basic setting of the hopper motor
      hopperConfig
        .smartCurrentLimit(HopperSubsystemConstants.kHopperCurrentLimit)
        .closedLoopRampRate(HopperSubsystemConstants.kHopperRampRate);

      /*
       * Configure the closed loop controller. We want to make sure we set the
       * feedback sensor as the primary encoder.
       */
      hopperConfig
          .closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control. We don't need to pass a closed
          // loop slot, as it will default to slot 0.
          .pid(HopperSubsystemConstants.kHopperKp, 
               HopperSubsystemConstants.kHopperKi, 
               HopperSubsystemConstants.kHopperKd)
          .outputRange(-1., 1)
          .maxMotion
          // Set MAXMotion parameters for position control
          .maxVelocity(100000)
          .maxAcceleration(300000)
          .allowedClosedLoopError(0.25);

      hopperConfig.idleMode(IdleMode.kBrake);
    }
  }
}