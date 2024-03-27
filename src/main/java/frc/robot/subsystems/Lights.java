// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.package frc.robot.subsystems;import edu.wpi.first.wpilibj2.command.SubsystemBase;
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Lights extends SubsystemBase {
  /** Creates a new SubsystemLights. */
  private AddressableLED lights;
  private AddressableLEDBuffer LEDBuffer;
  
  public Lights() {
    lights = new AddressableLED(6);
    LEDBuffer = new AddressableLEDBuffer(60);
    lights.setLength(60);
  }
  public void dataSetter(){
    lights.setData(LEDBuffer);
    lights.start();
  }
  public void setOneLightRGB(int index, int R, int G, int B){
    LEDBuffer.setRGB(index, R, G, B);
  }
  public void setLights(int start, int end, int R, int G, int B){
    for(int i = start; i < end; i++){
      setOneLightRGB(i, R, G, B);
    }
  }
  //All setLights methods are based on having four rows of fifteen lights, divided into three sections. 
  public void setLeft(int R, int G, int B){
    setLights(10, 20, R, G, B);
    setLights(40, 50, R, G, B);
  }

  public void setMid(int R, int G, int B){
   setLights(5, 10, R, G, B); 
   setLights(20, 25, R, G, B); 
   setLights(35, 40, R, G, B); 
   setLights(50, 55, R, G, B); 
  }

  public void setRight(int R, int G, int B){
    setLights(0, 5, R, G, B);
    setLights(25, 35, R, G, B);
    setLights(55, 60, R, G, B);
  }

  public void setSides(int R, int G, int B){
    setLeft(R, G, B);
    setRight(R, G, B);
  }

  public void lightsOut() {
    setLights(0, LEDBuffer.getLength(), 0, 0, 0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    dataSetter();
  }
}