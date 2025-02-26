package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//import java.io.ObjectInputFilter.Config;

import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {

    private SparkMax Leader;
    private SparkMax Follower;
    private XboxController m_functionsController;
    private final RelativeEncoder m_elevatorEncoder;

    public ElevatorSubsystem(XboxController controller) {
   
        Leader = new SparkMax(8, MotorType.kBrushless);
        Follower = new SparkMax(9, MotorType.kBrushless);
        m_elevatorEncoder = Leader.getAlternateEncoder();

        m_functionsController = controller;

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        SparkMaxConfig followConfig = new SparkMaxConfig();
        
        globalConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);
        
        leaderConfig
            .apply(globalConfig);

        followConfig
            .apply(globalConfig);
            //.inverted(true)
            //.follow(Leader);
    }

    // Method to set the speed of the motors
    public void setMotorSpeed(double speed) {
        Leader.set(speed);
        Follower.set(-speed);
    }

    // Method to stop the motors
    public void stopMotors() {
        Leader.set(0);
        Follower.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // You can add code here to update motor speeds based on controller input
        double speed = -m_functionsController.getLeftY();

        SmartDashboard.putNumber("Controller Y-axis",+ speed); //display the controller input on the SmartDashboard

        double scaledSpeed = speed * .15; //reduce motor speed to 50% of the controller input
        setMotorSpeed(scaledSpeed);

        SmartDashboard.putNumber("scaledSpeed", scaledSpeed); //display the scaled speed on the SmartDashboard

         // Display SparkMax information on the SmartDashboard
         SmartDashboard.putNumber("Leader Motor Speed", Leader.get());
         SmartDashboard.putNumber("Leader Motor Current", Leader.getOutputCurrent());
         SmartDashboard.putNumber("Leader Motor Temperature", Leader.getMotorTemperature());
 
         SmartDashboard.putNumber("Follower Motor Speed", Follower.get());
         SmartDashboard.putNumber("Follower Motor Current", Follower.getOutputCurrent());
         SmartDashboard.putNumber("Follower Motor Temperature", Follower.getMotorTemperature());
    }
}