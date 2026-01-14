// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CANRangeConstants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANrange;

public class RangeSensorSubsystem extends SubsystemBase {
  private final CANBus kCANBus = new CANBus("rio");
  private final CANrange m_canrange = new CANrange(CANRangeConstants.kCANRangeId, kCANBus);

  private double currentTime = Timer.getFPGATimestamp();

  /** Creates a new RangeSensorSubsystem. */
  public RangeSensorSubsystem() {
    m_canrange.getConfigurator().apply(Configs.CANRangeConfiguration.proximitySensorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Timer.getFPGATimestamp() - currentTime > CANRangeConstants.kUpdatePeriod) {
      // update the currentTime to ensure to stay within the refresh period
      currentTime += CANRangeConstants.kUpdatePeriod;

      var distance = m_canrange.getDistance();
      var signalStrength = m_canrange.getSignalStrength();

      /*
      System.out.println("Distance is " + distance.toString() + 
        " with a signal strength of " + signalStrength + " and " + 
        distance.getTimestamp().getLatency() + " seconds of latency");
      */
      
      // Get the isDetected StatusSignalValue without refreshing
      var isDetected = m_canrange.getIsDetected(false);
      /* This time wait for the signal to reduce latency */
      isDetected.waitForUpdate(CANRangeConstants.kUpdatePeriod); // Wait up to our period

      /*
      System.out.println(
        "Is Detected is " +
        isDetected.getValue() + " " +
        isDetected.getUnits() + " with " +
        isDetected.getTimestamp().getLatency() + " seconds of latency"
      );
      */

      /**
       * Notice when running this example that the second print's latency is always shorter than the first print's latency.
       * This is because we explicitly wait for the signal using the waitForUpdate() method instead of using the refresh()
       * method, which only gets the last cached value (similar to how Phoenix v5 works).
       * This can be used to make sure we synchronously update our control loop from the CAN bus, reducing any latency or jitter in
       * CAN bus measurements.
       * When the device is on a CANivore, the reported latency is very close to the true latency of the sensor, as the CANivore
       * timestamps when it receives the frame. This can be further used for latency compensation.
       */

      SmartDashboard.putNumber("CANRAnge/Distance (mm)", distance.getValueAsDouble());
      SmartDashboard.putNumber("CANRAnge/signalStrength", signalStrength.getValue());
      SmartDashboard.putBoolean("CANRAngeObject Detected", isDetected.getValue());
    }
  }

  public double getDistance(){
    return m_canrange.getDistance().getValueAsDouble();
  }
}
