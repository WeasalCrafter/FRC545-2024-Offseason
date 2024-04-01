package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANConstants;
import frc.robot.Constants.SpeedConstants;;

public class Climbers extends SubsystemBase{
    private CANSparkMax m_motor1;
    private CANSparkMax m_motor2;

    private double m_desiredSpeed = 0;
    private double m_climbSpeed = SpeedConstants.ClimberSpeed;

    public Climbers(){
        m_motor1 = new CANSparkMax(CANConstants.ACTUATOR_MOTOR_1, MotorType.kBrushless);
        m_motor2 = new CANSparkMax(CANConstants.ACTUATOR_MOTOR_2, MotorType.kBrushless);

        m_motor2.follow(m_motor1, false);
    }
    
    @Override
    public void periodic(){
        m_motor1.set(m_desiredSpeed);
        SmartDashboard.putNumber("Climber Speed", m_desiredSpeed);
        super.periodic();
    }

    public Command startClimbers(){
        return new InstantCommand(() -> {
            m_desiredSpeed = m_climbSpeed;
        }, this);
    }

    public Command reverseClimbers(){
        return new InstantCommand(() -> {
            m_desiredSpeed = -m_climbSpeed;
        }, this);
    }

    public Command stopClimbers(){
        return new InstantCommand(() -> {
            m_desiredSpeed = 0;
        }, this);
    }
}
