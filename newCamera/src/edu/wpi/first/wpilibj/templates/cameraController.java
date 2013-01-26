/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
/**
 *
 * @author Warbots
 */
public class cameraController {
  AxisCamera cam = null;
 CriteriaCollection cc = new CriteriaCollection();      // create the criteria for the particle filter
  
      public cameraController() {
        cam = AxisCamera.getInstance();
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, 10, 800, false); //looks for cetian size rectangles
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT, 10, 800, false);
       }
       
       public cameraController(int minWidth, int maxWidth,
       int minHeight, int maxHeight) {
        cam = AxisCamera.getInstance();
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_WIDTH, minWidth, maxWidth, false); //looks for cetian size rectangles
        cc.addCriteria(MeasurementType.IMAQ_MT_BOUNDING_RECT_HEIGHT, minHeight, maxHeight, false);
       }
       
       public ParticleAnalysisReport[] getParticles(int redMin, int redMax, //returns a list of particle reports form the camera
               int greenMin, int greenMax, 
               int blueMin, int blueMax) {   
        try {
                ColorImage image;     
                    image = cam.getImage();
                BinaryImage thresholdImage = image.thresholdRGB(redMin, redMax, greenMin, greenMax, blueMin, blueMax);   // keep certian colors
                BinaryImage bigObjectsImage = thresholdImage.removeSmallObjects(false, 2);  // remove small artifacts; tweak erosion amount
                BinaryImage convexHullImage = bigObjectsImage.convexHull(false);          // fill in occluded rectangles
                BinaryImage filteredImage = convexHullImage.particleFilter(cc);           // find filled in rectangles
                
                ParticleAnalysisReport[] reports = filteredImage.getOrderedParticleAnalysisReports();  // get list of results
            
                filteredImage.free();
                convexHullImage.free();
                bigObjectsImage.free();//THIS MUST BE DONE FOR MEM REASONS
                thresholdImage.free();
                image.free();
               return reports;
               
            } catch (NIVisionException ex) {
                ex.printStackTrace();
                return null;
             } catch (AxisCameraException ex) {
                    ex.printStackTrace();
                return null;
             }
       }
       
       public ParticleAnalysisReport getLargestParticle(int redMin, int redMax, //returns largest particle detected in the current camera image
               int greenMin, int greenMax, 
               int blueMin, int blueMax) {
            ParticleAnalysisReport[] reports = this.getParticles(redMin, redMax, greenMin, greenMax, blueMin, blueMax);
              int temp =0;
            double max = 0;
            for (int i = 0; i < reports.length; i++) {                                // print results
                    ParticleAnalysisReport r = reports[i];
                        if(reports[i].particleArea > max){
                            max = reports[i].particleArea;
                            temp = i;
                        }    
               }
            if(reports.length > 0) {
                return reports[temp];
            }
            else {
                return null;
            }
       }
}
