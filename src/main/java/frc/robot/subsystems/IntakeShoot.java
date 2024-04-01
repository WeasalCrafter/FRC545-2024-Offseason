package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SpeedConstants;
import frc.robot.Constants.CANConstants;;

public class IntakeShoot extends SubsystemBase{
    private CANSparkMax m_intake1;
    private CANSparkMax m_intake2;
    private CANSparkMax m_shooter1;
    private CANSparkMax m_shooter2;
    private CANSparkMax m_support1;
    private CANSparkMax m_support2;
    
    private double m_intakeSpeed = SpeedConstants.IntakeSpeed;
    private double m_humanIntakeSpeed = SpeedConstants.HumanIntakeSpeed;
    private double m_ampShooterSpeed = SpeedConstants.AmpShooterSpeed;
    private double m_speakerShooterSpeed = SpeedConstants.SpeakerShooterSpeed;

    private double m_speakerOutakeDelay = SpeedConstants.SpeakerOutakeDelay;
    private double m_speakerShooterDelay = SpeedConstants.SpeakerShooterDelay;

    private double m_desiredIntakeSpeed = 0;
    private double m_desiredShooterSpeed = 0;
    private double m_desiredSupportSpeed = 0;

    public IntakeShoot(){
        m_intake1 = new CANSparkMax(CANConstants.INTAKE_MOTOR_1, CANSparkMax.MotorType.kBrushless);
        m_intake2 = new CANSparkMax(CANConstants.INTAKE_MOTOR_2, CANSparkMax.MotorType.kBrushless);

        m_shooter1 = new CANSparkMax(CANConstants.SHOOTER_MOTOR_1, CANSparkMax.MotorType.kBrushless);
        m_shooter2 = new CANSparkMax(CANConstants.SHOOTER_MOTOR_2, CANSparkMax.MotorType.kBrushless);

        m_support1 = new CANSparkMax(CANConstants.SUPPORT_MOTOR_1, CANSparkMax.MotorType.kBrushless);
        m_support2 = new CANSparkMax(CANConstants.SUPPORT_MOTOR_2, CANSparkMax.MotorType.kBrushless);

        m_intake2.follow(m_intake1, true);
        m_shooter2.follow(m_shooter1, true);
        m_support2.follow(m_support1, true);
    }

    @Override
    public void periodic() {
        m_intake1.set(m_desiredIntakeSpeed);
        m_shooter1.set(m_desiredShooterSpeed);
        m_support1.set(m_desiredSupportSpeed);

        SmartDashboard.putNumber("Intake Speed", m_desiredIntakeSpeed);
        SmartDashboard.putNumber("Shooter Speed", m_desiredShooterSpeed);
        SmartDashboard.putNumber("Support Speed", m_desiredSupportSpeed);
        super.periodic();
    }

    public Command startIntake(){
        return new InstantCommand(() -> {
            m_desiredIntakeSpeed = m_intakeSpeed;
            m_desiredSupportSpeed = m_intakeSpeed;
        }, this);
    }

    public Command startOutake(){
        return new InstantCommand(() -> {
            m_desiredIntakeSpeed = -m_intakeSpeed;
            m_desiredSupportSpeed = -m_intakeSpeed;
        }, this);
    }

    public Command stopIntake(){
        return new InstantCommand(() -> {
            m_desiredIntakeSpeed = 0;
            m_desiredSupportSpeed = 0;
        }, this);
    }

    public Command startAmpShot(){
        return new InstantCommand(() -> {
            m_desiredShooterSpeed = m_ampShooterSpeed;
            m_desiredSupportSpeed = m_ampShooterSpeed;
        }, this);
    }

    public Command startSpeakerShot(){
        return new InstantCommand(() -> {
            m_desiredShooterSpeed = m_speakerShooterSpeed;
            m_desiredSupportSpeed = m_speakerShooterSpeed;

        }, this);
    }

    public Command startHumanIntake(){
        return new InstantCommand(() -> {
            m_desiredShooterSpeed = m_humanIntakeSpeed;
            m_desiredSupportSpeed = m_humanIntakeSpeed;
        }, this);
    }

    public Command stopShooter(){
        return new InstantCommand(() -> {
            m_desiredShooterSpeed = 0;
            m_desiredSupportSpeed = 0;
        }, this);
    }

    public Command stopAll(){
        return new InstantCommand(() -> {
            m_desiredIntakeSpeed = 0;
            m_desiredShooterSpeed = 0;
            m_desiredSupportSpeed = 0;
        }, this);
    }

    public SequentialCommandGroup tempSpeakerOuttake(){
        return new SequentialCommandGroup(
            startOutake(),
            new WaitCommand(m_speakerOutakeDelay),
            stopIntake()
        );
    }

    public SequentialCommandGroup tempSpeakerShooter(){
        return new SequentialCommandGroup(
            startSpeakerShot(),
            new WaitCommand(m_speakerShooterDelay),
            stopShooter()
        );
    }

    public SequentialCommandGroup fullSpeakerShot(){
        return new SequentialCommandGroup(
            tempSpeakerOuttake(),
            new WaitCommand(0.05),
            tempSpeakerShooter()
        );
    }

}

