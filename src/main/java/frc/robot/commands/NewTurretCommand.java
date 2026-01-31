package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;

public class NewTurretCommand extends Command {

    private final TurretSubsystem turret;
    private final CommandSwerveDrivetrain drivetrain;

    private static final double targetX = 1.82;
    private static final double targetY = 1.58;

    private final PIDController pid = new PIDController(0.03, 0, 0);

    public NewTurretCommand(TurretSubsystem turret,CommandSwerveDrivetrain drivetrain) {
        this.turret = turret;
        this.drivetrain = drivetrain;

        pid.enableContinuousInput(-180, 180);

        addRequirements(turret);
    }

    @Override
    public void execute() {

        Pose2d robotPose = drivetrain.getCurrentPose();

        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();

        // Angle from robot to target (field-relative)
        double targetFieldAngle = Math.toDegrees(Math.atan2(dy, dx));

        // Robot heading (field-relative)
        double robotHeading = robotPose.getRotation().getDegrees();

        // Convert to turret-relative angle
        double desiredTurretAngle = targetFieldAngle - robotHeading;

        // Wrap to [-180, 180]
        desiredTurretAngle = Math.IEEEremainder(desiredTurretAngle, 360);

        //double currentTurretAngle = turret.getAngle();
        double turretOffset = 90; // example 
        double currentTurretAngle = turret.getAngle(20) + turretOffset;
        //currentTurretAngle = Math.IEEEremainder(currentTurretAngle, 360);

        double output = pid.calculate(currentTurretAngle, desiredTurretAngle);
        output = MathUtil.clamp(output, -0.5, 0.5);

        SmartDashboard.putNumber("Ttest/Targetx" , targetX);
        SmartDashboard.putNumber("Ttest/Targety" , targetY);
        SmartDashboard.putNumber("Ttest/RobotPoseX" , robotPose.getX());
        SmartDashboard.putNumber("Ttest/RobotPoseY" , robotPose.getY());
        SmartDashboard.putNumber("Ttest/targetFieldAngle" , targetFieldAngle);
        SmartDashboard.putNumber("Ttest/robotHeading" , robotHeading);
        SmartDashboard.putNumber("Ttest/desiredTurretAngle" , desiredTurretAngle);
        SmartDashboard.putNumber("Ttest/currentTurretAngle" , currentTurretAngle);
        SmartDashboard.putNumber("Ttest/PIDOutput" , output);
        



        turret.setMotor(output);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
