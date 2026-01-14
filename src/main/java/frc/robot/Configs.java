package frc.robot;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.CANRangeConstants;
import frc.robot.Constants.TurretSubsystemConstants;

public class Configs {

 
    /* *****************
    * TurretSubsystem 
    */
   public static final class TurretSubsystemConfiguration {
     public static final SparkMaxConfig turretConfig = new SparkMaxConfig();
 
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
           .maxVelocity(2000)
           .maxAcceleration(10000)
           .allowedClosedLoopError(0.25);
 
       turretConfig.idleMode(IdleMode.kBrake);
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

  
}