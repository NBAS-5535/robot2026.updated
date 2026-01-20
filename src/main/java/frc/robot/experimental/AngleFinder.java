package frc.robot.experimental;

import frc.robot.subsystems.VisionSubsystem;

public class AngleFinder {
    private VisionSubsystem m_vision;
    private double myAngle1 = 0.0;
    private double myAngle2 = 0.0;
    private double distance = 0.0;
    private double initialVelo = 10.0;
    private static double gravity = 9.81;
    private double targetheight = 10.0;

    AngleFinder(VisionSubsystem vision){
        this.m_vision = vision;
    }

    public void getCameraDistance(){
        this.distance = m_vision.getDistance();
    }

    public void FindmyAngle(){
        myAngle1 = Math.atan((Math.pow(initialVelo,2)) + Math.sqrt(Math.pow(initialVelo,4) - gravity * ((gravity * Math.pow(distance,2)) + (2 * targetheight * Math.pow(initialVelo,2))))) / gravity * distance;
        myAngle2 = Math.atan((Math.pow(initialVelo,2)) - Math.sqrt(Math.pow(initialVelo,4) - gravity * ((gravity * Math.pow(distance,2)) + (2 * targetheight * Math.pow(initialVelo,2))))) / gravity * distance;
    }
}
