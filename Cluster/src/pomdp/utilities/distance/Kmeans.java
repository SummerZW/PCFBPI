package pomdp.utilities.distance;

import pomdp.environments.POMDP;
import pomdp.utilities.*;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Iterator;
import java.util.Map;

public class Kmeans {

    private static int k = 0;
    private static ArrayList<BeliefState> bs = null;
    private double deviation = 0.01;

    public Kmeans(int k, BeliefStateVector<BeliefState> vBeliefPoints) {
        this.k = k;
        this.bs = new ArrayList<>();
        for (int i = 0; i < vBeliefPoints.size(); i++) {
            this.bs.add(vBeliefPoints.get(i));
        }
    }

    public ArrayList<BeliefStateVector<BeliefState>> doKMEANS() {
        // 初始化以及第一次选择质心
        ArrayList<BeliefStateVector<BeliefState>> bsvList = new ArrayList<>();
        for (int i = 0; i < k; i++) {
            bsvList.add(new BeliefStateVector<>());
        }
        ArrayList<BeliefState> centers = this.randomSelect();
        for (BeliefState b : this.bs) {
            int centerNear = (int) this.mostNear(b, centers)[0];
            bsvList.get(centerNear).add(b);
        }

        while (true) {
            double mistake = 0.0;

            // 计算质心
            ArrayList<BeliefState> newCenters = new ArrayList<>(); // 新的质心
            for (BeliefStateVector<BeliefState> bsv : bsvList) {
                BeliefState newCenter = this.calCentroid(bsv);
                mistake += this.mostNear(newCenter, centers)[1];
                newCenters.add(newCenter);
            }

            Logger.getInstance().logln(mistake);
            // 判断前后两次质心差
            if (mistake < this.deviation) {
                break;
            }

            // 新的聚类list
            ArrayList<BeliefStateVector<BeliefState>> newBsvList = new ArrayList<>();
            for (int i = 0; i < k; i++) {
                newBsvList.add(new BeliefStateVector<>());
            }

            // 将所有信念点重新加入list
            for (BeliefState b : this.bs) {
                int centerNear = (int) this.mostNear(b, centers)[0];
                newBsvList.get(centerNear).add(b);
            }

            // 更新
            centers = newCenters;
            bsvList = newBsvList;

        }
        return bsvList;
    }

    /**
     * 随机选择k个信念状态
     */
    private ArrayList<BeliefState> randomSelect() {
        Collections.shuffle(this.bs);
        ArrayList<BeliefState> result = new ArrayList<>();
        for (int i = 0; i < this.k; i++) {
            result.add(this.bs.get(i));
        }
        return result;
    }

    /**
     * 在一组信念状态中查找与特点信念状态最近的信念状态
     */
    public static double[] mostNear(BeliefState b, ArrayList<BeliefState> centers) {
        L1Distance l1Distance = new L1Distance();
        double minDistance = Double.MAX_VALUE;
        int near = -1;
        for (int i = 0; i < centers.size(); i++) {
            double distance = l1Distance.distance(b, centers.get(i));
            if (distance < minDistance) {
                minDistance = distance;
                near = i;
            }
        }
        return new double[]{near, minDistance};
    }

    /**
     * 计算一组信念状态的质心
     */
    public static BeliefState calCentroid(BeliefStateVector<BeliefState> bsv) {
        POMDP pomdp = bsv.get(0).getBeliefStateFactory().getPOMDP();
        int cState = pomdp.getStateCount();
        double[] proSum = new double[cState];
        for (int i = 0; i < cState; i++) {
            proSum[i] = 0.0;
        }
        for (BeliefState bs : bsv) {
            Iterator<Map.Entry<Integer, Double>> itNonZero = bs.getNonZeroEntries().iterator();
            int iState = -1;
            while (iState < Integer.MAX_VALUE) {
                LDistance.Belief b = new LDistance.Belief(itNonZero);
                iState = b.iState;
                if (b.dValue > 0.0001) {
                    proSum[iState] += b.dValue;
                }
            }
        }
        for (int i = 0; i < cState; i++) {
            proSum[i] /= bsv.size();
        }
        BeliefState bs = new TabularBeliefState(pomdp.getStateCount(), pomdp.getActionCount(), pomdp.getObservationCount(), 0, false, false, new BeliefStateFactory(pomdp));
        bs.setPro(proSum);
        return bs;
    }
}
