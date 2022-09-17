package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexConstants;
import com.revrobotics.CANSparkMaxLowLevel;

public class Index extends SubsystemBase {
  private CANSparkMax indexMotor;
  private CANSparkMax rollerMotor;
  private DigitalInput indexBeam;
  private DigitalInput rollerBeam;

  public enum IndexState {
    SHOOTING,
    INTAKING,
    EXTAKING,
    DEFAULT
    }

    private IndexState indexState = IndexState.DEFAULT;

  /** Creates a new Index. */
  public Index() {
    indexMotor = new CANSparkMax(IndexConstants.kIndexMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    rollerMotor = new CANSparkMax(IndexConstants.kRollerMotorPort, CANSparkMaxLowLevel.MotorType.kBrushless);
    indexBeam = new DigitalInput(IndexConstants.kIndexBeamChannel);
    rollerBeam = new DigitalInput(IndexConstants.kRollerBeamChannel);
}

  public void setRollerPower(double power){
    rollerMotor.set(power);
  }

  public void setIndexPower(double power){
    indexMotor.set(power);
  }

  public void moveAll(double power){
      indexMotor.set(power);
      rollerMotor.set(power);
  }

  public void disable() {
      moveAll(0);
      setIndexState(IndexState.DEFAULT);
  }

  public boolean getIndexBreakbeam() {

   if(indexBeam.get()) {
       return true;
   }
   else {
        return false;
    }
  }

  public boolean getRollerBreakbeam() {
    if(rollerBeam.get()) {
        return true;
    }
    else {
         return false;
    }
  }

  public void setIndexState(IndexState indexState) {
    this.indexState = indexState;
  }

  public void IndexMotion() {
      if (indexState == IndexState.SHOOTING) {
          moveAll(.5);
      }
      if (indexState == IndexState.EXTAKING) {
          moveAll(-.5);
      }
      if (indexState == IndexState.INTAKING){
          if(getIndexBreakbeam() == true) {
              setIndexPower(0);
              setRollerPower(.5);
          }
          if(getIndexBreakbeam() & getRollerBreakbeam() == true) {
              moveAll(.5);
          }
          else {
              moveAll(.5);
          }
      }
  }



  @Override
  public void periodic() {
    
  }
}