package dbscan;

/**
 * Created with IntelliJ IDEA.
 * User: hanbing
 * Date: 14-2-17
 * Time: 涓嬪崍5:29
 * To change this template use File | Settings | File Templates.
 */
import java.io.IOException;
import java.util.*;

import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateVector;

public class Dbscan {
    //tiger-grid:0.3 103,0.35 30,0.4 11,0.5 3,
    private static double e=0.5;//蔚鍗婂緞
    private static int minp=4;//瀵嗗害闃堝�
    private static List<Point> pointsList=new ArrayList<Point>();//瀛樺偍鍘熷鏍锋湰鐐�
    public static List<List<Point>> resultList=new ArrayList<List<Point>>();//瀛樺偍鏈�悗鐨勮仛绫荤粨鏋�
    public static List<Point> unclusteredPointList = new ArrayList<Point>();

    
    /**
     * Warning: Do not support unclusteredPointList
     * @param pointsetPath
     * @throws IOException
     */
    public static void applyDbscanByFile(String pointsetPath) throws IOException{
        //e:\pomdp\src\dbscan\
        pointsList=Utility.getPointsList(pointsetPath);
        System.out.println(pointsList.size());
        for(int index=0;index<pointsList.size();++index){
            System.out.println("point:"+index);
            List<Point> tmpLst=new ArrayList<Point>();
            Point p=pointsList.get(index);
            if(p.isClassed())
                continue;
            tmpLst=Utility.isKeyPoint(pointsList, p, e, minp);
            if(tmpLst!=null){
                resultList.add(tmpLst);
            }
        }
        int length=resultList.size();
        System.out.println(length);
        for(int i=0;i<length;++i){
            System.out.println(i);
            for(int j=0;j<length;++j){
                if(i!=j){
                    if(Utility.mergeList(resultList.get(i), resultList.get(j))){
                        resultList.get(j).clear();
                    }
                }
            }
        }
    }
    
    public static void applyDbscan(BeliefStateVector<BeliefState> beliefStateVector,int stateCount) throws IOException{
        pointsList=Utility.getPointsListFromBeliefStateVector(beliefStateVector,stateCount);
        System.out.println(pointsList.size());
        for(int index=0;index<pointsList.size();++index){
            System.out.println("point:"+index);
            List<Point> tmpLst=new ArrayList<Point>();
            Point p=pointsList.get(index);
            if(p.isClassed())
                continue;
            tmpLst=Utility.isKeyPoint(pointsList, p, e, minp);
            if(tmpLst!=null){
                resultList.add(tmpLst);
            }
        }
        
        int length=resultList.size();
        System.out.println(length);
        for(int i=0;i<length;++i){
            System.out.println(i);
            for(int j=0;j<length;++j){
                if(i!=j){
                    if(Utility.mergeList(resultList.get(i), resultList.get(j))){
                        resultList.get(j).clear();
                    }
                }
            }
        }
        
        HashSet<Point> hs = new HashSet<Point>(beliefStateVector.size()/10);
        
        for(List<Point> points:resultList){
        	hs.addAll(points);
        }
       

        for(Point p:pointsList){
            if(!hs.contains(p)){
                unclusteredPointList.add(p);//找到没有成簇的点，即散点
            }
        }
    }
    public static void newapplyDbscan(String pointsetPath,double newe,int newminp) throws IOException{
        Dbscan.e=newe;
        Dbscan.minp=newminp;
        //e:\pomdp\src\dbscan\
        pointsList=Utility.getPointsList(pointsetPath);
        for(int index=0;index<pointsList.size();++index){
            List<Point> tmpLst=new ArrayList<Point>();
            Point p=pointsList.get(index);
            if(p.isClassed())
                continue;
            tmpLst=Utility.isKeyPoint(pointsList, p, e, minp);
            if(tmpLst!=null){
                resultList.add(tmpLst);
            }
        }
        int length=resultList.size();
        for(int i=0;i<length;++i){
            for(int j=0;j<length;++j){
                if(i!=j){
                    if(Utility.mergeList(resultList.get(i), resultList.get(j))){
                        resultList.get(j).clear();
                    }
                }
            }
        }
    }
    public static void main(String[] args) {
        try {

            for(double i=0;i<1.5;i+=0.05){
                //璋冪敤DBSCAN鐨勫疄鐜扮畻娉�
                newapplyDbscan("pointset", i, 40);
                //System.out.println("here:"+resultList.size());
                double count=0;
                for(List ml:resultList){
                    if(ml.size()>0){
                        count++;
                    }
                }
                //Utility.display(resultList);
                System.out.println(i+" "+count+" "+resultList.size());
                resultList.clear();
            }

        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

    }

}

