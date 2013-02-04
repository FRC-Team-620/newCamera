/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author Warbots
 */
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;

public class PIDcntrl {
    Jaguar j1 = null; //new Jaguar(channel1);
    Jaguar j2 = null; //new Jaguar(channel2);
    
    public PIDcntrl(int channelL, int channelR){
        j1 = new Jaguar(channelL);
        j2 = new Jaguar(channelR);
    }
    

    public void loop(int center, ParticleAnalysisReport r){
        if(r != null){
        j1.set((320-r.center_mass_x)/50);
        j2.set((320-r.center_mass_x)/50);
        }else{
            j1.set(.3);
            j2.set(.3);
        }           
        }
}
