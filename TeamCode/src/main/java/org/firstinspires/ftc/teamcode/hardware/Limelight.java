package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Config.*;

import java.util.List;
import java.util.Set;

public class Limelight {
    private Limelight3A limelight;
    private LLResult currentResult;
    private LLResult lastGoodResult;
    private LLResult finalResult;

    private long lastGoodTime = 0;

    public boolean tVisible = false;
    public double tx;
    public double ty;

    public Limelight(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT);
        limelight.pipelineSwitch(0); //set pipeline index
        limelight.setPollRateHz(100);
        start();
    }

    public void start(){limelight.start();}

    public void update(Alliance.Hive activeHive){
        currentResult = limelight.getLatestResult();//fetch info packet
        if (currentResult != null && currentResult.isValid()){ //if it currently sees a tag
            lastGoodResult = currentResult;
            lastGoodTime = System.currentTimeMillis(); //start a stopwatch
        }

        if (lastGoodResult != null && System.currentTimeMillis() - lastGoodTime <= RESULT_TIMEOUT_MS){ //if the stopwatch is less than 1 second
            //use the last good result
            finalResult = lastGoodResult;
        }else{
            //has not seen an april tag for more than 1 second!
            finalResult = currentResult;
        }
        tVisible = getBestTVisible(activeHive);
        tx = getBestTX(activeHive);
        ty = lastGoodResult.getTy();
    }

    public boolean isActiveHiveTag(int id, Alliance.Hive activeHive){ return activeHive.allTagIds.contains(id);}

    public boolean isMiddleTag(int id, Alliance.Hive activeHive){return activeHive.middleTagIds.contains(id);}

    public boolean getBestTVisible(Alliance.Hive activeHive){
        if (finalResult == null) { //safety checks in case finalResult is null
            return false;
        }
        List<LLResultTypes.FiducialResult> fiducials = finalResult.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {//return 0 if no tags
            return false;
        }
        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            if (isActiveHiveTag(fiducial.getFiducialId(), activeHive)) {
                return true;
            }
        }
        return false;
    }

    //it gets the best tx using the following algorithm:
    //always prioritize returning the tx positions of middle tags
    //maintain outer tags as backups in case no middle tags are seen
    //if AT LEAST two middle tags are seen it will average together the tx
    //otherwise if it sees two outer tags it will average together the tx
    public double getBestTX(Alliance.Hive activeHive){
        if (finalResult == null) { //safety checks in case finalResult is null
            return 0;
        }
        List<LLResultTypes.FiducialResult> fiducials = finalResult.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {//return 0 if no tags
            return 0;
        }

        double average = 0;
        int middleTagCount = 0;//needs to be two in order to average (should only ever be max of two)
        int backupTagCount = 0;
        double backupAverage = 0;

        double optimalTX = Double.NaN;
        double backupTX = 0;
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();
            if (!isActiveHiveTag(id, activeHive)){
                //random ah tag from nowhere so we ignore it completely
                continue;
            }

            double tx = fiducial.getTargetXDegrees();
            backupTX = tx;

            if(isMiddleTag(id, activeHive)){
                //it must be an april on the inside
                optimalTX = tx;
                middleTagCount++;
                average += tx;
            }else{
                //it must be an april tag on the outside
                backupTagCount++;
                backupAverage += tx;
            }
        }

        if (middleTagCount >= 2){
            optimalTX = average/middleTagCount;
        } else if (backupTagCount >= 2) {
            optimalTX = backupAverage/backupTagCount;
        }
        if (Double.isNaN(optimalTX)){
            optimalTX = backupTX;
        }
        return optimalTX;
    }

    public void stop(){limelight.stop();}
}
