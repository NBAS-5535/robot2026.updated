package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;

public class NewTurretCommand extends Command {

    private final TurretSubsystem turret;
    private final CommandSwerveDrivetrain drivetrain;

    private static final double targetX = 4.55;
    private static final double targetY = 3.97;

    private final PIDController pid = new PIDController(0.025, 0, 0);

    public NewTurretCommand(TurretSubsystem turret,CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;

        pid.enableContinuousInput(-180, 180);

        addRequirements(turret);
    }

     @Override
    public void initialize() {
        pid.setTolerance(10);
  }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getCurrentPose();
        double rotationangle = robotPose.getRotation().getDegrees();
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();

        // Angle from robot to target (field-relative)
        double targetFieldAngle = Math.toDegrees(Math.atan2(dy, dx));
        double desiredTurretAngle = MathUtil.inputModulus( targetFieldAngle - rotationangle, -180, 180 );
        turret.rotate(desiredTurretAngle,rotationangle);

        SmartDashboard.putNumber("Ttest/RotationAngle", rotationangle);
        SmartDashboard.putNumber("Ttest/TargetFieldAngle", targetFieldAngle);
        SmartDashboard.putNumber("Ttest/TurretAngle", turret.getFieldRelativeTurretAngle(rotationangle));
        SmartDashboard.putNumber("Ttest/Targetx" , targetX);
        SmartDashboard.putNumber("Ttest/Targety" , targetY);
        SmartDashboard.putNumber("Ttest/RobotPoseX" , robotPose.getX());
        SmartDashboard.putNumber("Ttest/RobotPoseY" , robotPose.getY());

       
        
        // Robot heading (field-relative)
        /* 
        double robotHeading = robotPose.getRotation().getDegrees();

        // Convert to turret-relative angle
        double desiredTurretAngle = targetFieldAngle - robotHeading;
        desiredTurretAngle = MathUtil.inputModulus(desiredTurretAngle, -180, 180);
        double turretOffset = 45; // example 
        double motorRotations = turret.getAngle(); // raw encoder rotations 
        double turretDegrees = (motorRotations / 20.0) * 360.0; // Wrap to [-180, 180] 
        double currentTurretAngle = MathUtil.inputModulus(turretDegrees + turretOffset, -180, 180);
        double output = pid.calculate(currentTurretAngle, desiredTurretAngle);
        output = MathUtil.clamp(output, -0.25, 0.25);

        

        SmartDashboard.putNumber("Ttest/Targetx" , targetX);
        SmartDashboard.putNumber("Ttest/Targety" , targetY);
        SmartDashboard.putNumber("Ttest/RobotPoseX" , robotPose.getX());
        SmartDashboard.putNumber("Ttest/RobotPoseY" , robotPose.getY());
        SmartDashboard.putNumber("Ttest/targetFieldAngle" , targetFieldAngle);
        SmartDashboard.putNumber("Ttest/robotHeading" , robotHeading);
        SmartDashboard.putNumber("Ttest/desiredTurretAngle" , desiredTurretAngle);
        SmartDashboard.putNumber("Ttest/currentTurretAngle" , currentTurretAngle);
        SmartDashboard.putNumber("Ttest/PIDOutput" , output);
        SmartDashboard.putNumber("Ttest/TurretRawAngle", turret.getAngle());
        SmartDashboard.putNumber("Ttest/TurretDegrees", turretDegrees);
        SmartDashboard.putNumber("Ttest/MotorRotations", motorRotations);
        SmartDashboard.putNumber("Ttest/CurrentTurretAngle", currentTurretAngle);

        turret.setMotor(output);
        */
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
