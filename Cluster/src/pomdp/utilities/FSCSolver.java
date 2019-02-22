package pomdp.utilities;


import pomdp.environments.POMDP;
import pomdp.utilities.distance.L1Distance;

import java.io.*;
import java.math.BigDecimal;
import java.util.*;
import java.util.Map.Entry;

public class FSCSolver {

    private BeliefStateVector<BeliefState> vBeliefPoints;
    private ArrayList<MachineState> fsc = new ArrayList<>();
    private POMDP m_pomdp;

    public FSCSolver(BeliefStateVector<BeliefState> vBeliefPoints, POMDP m_pomdp) {
        this.vBeliefPoints = vBeliefPoints;
        this.m_pomdp = m_pomdp;
    }

    public FSCSolver() {

    }

    public void initialFSC(double gamma) {
        ArrayList<MachineState> tempfsc = new ArrayList<>();
        // 初始化每个动作对应的machine state
        for (int i = 0; i < m_pomdp.getActionCount(); i++) {
            MachineState tempMS = new MachineState();
            tempMS.setAction(i);
            AlphaVector vec = new TabularAlphaVector(null, i, m_pomdp);
            vec = this.initialWithBlindPolicy(vec, gamma);
            tempMS.setVec(vec);
            tempfsc.add(tempMS);
        }

        this.getPointsGrouped(tempfsc);
        this.deleteMSwithNoPoint(tempfsc);
        this.completeLink(tempfsc);
        this.setSamplePoints(tempfsc);
        this.fsc = tempfsc;

        System.out.println("initialize complete!");
        System.out.println("fsc size = " + fsc.size());
    }

    /**
     * 使用blind策略进行初始化
     */
    public AlphaVector initialWithBlindPolicy(AlphaVector vector, double gamma) {
        for (int iAction = 0; iAction < m_pomdp.getActionCount(); ++iAction) {
            for (int iState = 0; iState < m_pomdp.getStateCount(); ++iState) {
                double dSum = 0.0;
                // iState为起始状态，itNonZero中为终止状态
                Iterator<Entry<Integer, Double>> itNonZero = m_pomdp.getNonZeroTransitions(iState, iAction);
                while (itNonZero.hasNext()) {
                    Entry<Integer, Double> entry = itNonZero.next();
                    int iEndState = entry.getKey().intValue();
                    double dTr = entry.getValue().doubleValue();
                    double dValue = vector.valueAt(iEndState);
                    dSum += dTr * dValue;
                }
                double dReward = m_pomdp.R(iState, iAction);
                double dNewValue = dReward + dSum * gamma;
                // 将对应的状态与值存入向量
                vector.setValue(iState, dNewValue);
            }
        }
        return vector;
    }

    /**
     * 统治域
     */
    public void getPointsGrouped(ArrayList<MachineState> fsc) {
        // 对于每一个信念点，加入到一个MachineState中
        for (BeliefState bs : vBeliefPoints) {
            int idx = -1;
            double value = -Double.MAX_VALUE;
            for (int k = 0; k < fsc.size(); k++) {
                if ((fsc.get(k).getVec().dotProduct(bs) > value)) {
                    value = fsc.get(k).getVec().dotProduct(bs);
                    idx = k;
                }
            }
            fsc.get(idx).pointGroup.add(bs);
            bs.setMs(fsc.get(idx));
        }
    }

    /**
     * 删除没有统治域的machine state
     */
    private void deleteMSwithNoPoint(ArrayList<MachineState> tempfsc) {
        for (int i = tempfsc.size() - 1; i >= 0; i--) {
            MachineState ms = tempfsc.get(i);
            if (ms.pointGroup.size() == 0) {
                tempfsc.remove(ms);
            }
        }
    }

    // TODO Why first point?
    private void completeLink(ArrayList<MachineState> tempfsc) {
        for (int i = 0; i < tempfsc.size(); i++) {
            BeliefState bs = tempfsc.get(i).pointGroup.get(0);
            int action = tempfsc.get(i).getAction();
            for (int k = 0; k < m_pomdp.getObservationCount(); k++) {
                BeliefState nextPoint;
                nextPoint = bs.nextBeliefState(action, k);
                if (nextPoint == null) continue;
                int outwardIdx = getMaxValueMachineStateIDX(tempfsc, nextPoint);
                tempfsc.get(i).outwardLink.put(k, outwardIdx);
            }
        }
    }

    public int getMaxValueMachineStateIDX(ArrayList<MachineState> fsc, BeliefState point) {
        int idx = -1;
        double value = -Double.MAX_VALUE;
        for (int i = 0; i < fsc.size(); i++) {
            if (fsc.get(i).getVec().dotProduct(point) > value) {
                value = fsc.get(i).getVec().dotProduct(point);
                idx = i;
            }
        }
        return idx;
    }

    private void setSamplePoints(ArrayList<MachineState> fsc) {
        for (MachineState aFsc : fsc) {
            BeliefStateVector<BeliefState> tempSP;
            tempSP = aFsc.getSamplePoints();

            for (int k = 0; k < m_pomdp.getStateCount() + 2; k++) {
                tempSP.add(aFsc.pointGroup.get(0));
            }

            for (int k = 0; k < aFsc.getPointGroup().size(); k++) {
                for (int m = 0; m < m_pomdp.getStateCount(); m++) {
                    if (aFsc.getPointGroup().get(k).valueAt(m) > tempSP.get(m).valueAt(m)) {
                        tempSP.set(m, aFsc.pointGroup.get(k));
                    }
                }
                if (aFsc.getVec().dotProduct(aFsc.getPointGroup().get(k))
                        > aFsc.getVec().dotProduct(tempSP.get(m_pomdp.getStateCount()))) {
                    tempSP.set(m_pomdp.getStateCount(), aFsc.pointGroup.get(k));
                }
                if (aFsc.getVec().dotProduct(aFsc.getPointGroup().get(k))
                        < aFsc.getVec().dotProduct(tempSP.get(m_pomdp.getStateCount() + 1))) {
                    tempSP.set(m_pomdp.getStateCount() + 1, aFsc.pointGroup.get(k));
                }

            }
        }
    }

    public boolean isAllMarked(ArrayList<BeliefStateVector<BeliefState>> result) {
        for (BeliefStateVector<BeliefState> pointList : result) {
            if (!pointList.isEmpty())
                return false;
        }
        return true;
    }

    public void removeAllMarks() {
        for (BeliefState bs : vBeliefPoints) {
            bs.setUnmarked();
        }
    }

    public ArrayList<BeliefState> samplePoints(ArrayList<BeliefStateVector<BeliefState>> dbscanResult, double epsilon) {
        ArrayList<BeliefState> result = new ArrayList<>();
        for (BeliefStateVector<BeliefState> bsv : dbscanResult) {
            if (bsv.isEmpty())
                continue;

            Random random = new Random(System.nanoTime());
            int index = random.nextInt(bsv.size());//���ѡһ����
            BeliefState bs = bsv.get(index);
            result.add(bs);
            bsv.remove(index);

            L1Distance distancer = new L1Distance();
            for (int i = 0; i < bsv.size(); ++i) {
                if (distancer.distance(bs, bsv.get(i)) < epsilon)//ȥ��epsilon��Χ�ڵĵ�
                    bsv.remove(i);
            }
        }
        System.out.println("samplePoints complete!");
        return result;
    }

    public double newSumAZ(int stateIdx, int action, ArrayList<MachineState> fsc, BeliefState point) {
        double result = 0;

        for (int transToStateIdx = 0; transToStateIdx < m_pomdp.getStateCount(); transToStateIdx++) {

            for (int observ = 0; observ < m_pomdp.getObservationCount(); observ++) {
                BeliefState nextPoint = point.nextBeliefState(action, observ);
                if (nextPoint == null)
                    continue;
                double tr = m_pomdp.tr(stateIdx, action, transToStateIdx);
                double o = m_pomdp.O(action, transToStateIdx, observ);
                int index = getMaxValueMachineStateIDX(fsc, nextPoint);
                AlphaVector vec = fsc.get(index).getVec();
                double value = vec.valueAt(transToStateIdx);
                result += tr * o * value;

            }
        }
        return result;
    }

    private ArrayList maxVecAtPoint(ArrayList<AlphaVector> vecList, BeliefState point) {
        ArrayList al = new ArrayList();
        double value = -Double.MAX_VALUE;
        int idx = -1;
        for (int i = 0; i < vecList.size(); i++) {
            if (vecList.get(i).dotProduct(point) > value) {
                idx = i;
                value = vecList.get(i).dotProduct(point);
            }
        }
        al.add(idx);
        al.add(vecList.get(idx));
        return al;

    }

    AlphaVector getMaxAlpha(BeliefState bs, ArrayList<MachineState> fsc) {
        double maxValue = Double.NEGATIVE_INFINITY;
        AlphaVector avMaxAlpha = null;

        for (MachineState ms : fsc) {
            AlphaVector avCurrent = ms.getVec();
            double value = avCurrent.dotProduct(bs);

            if (value >= maxValue) {
                maxValue = value;
                avMaxAlpha = avCurrent;
            }
        }

        if (avMaxAlpha != null) {
            bs.setMaxValue(maxValue);
            bs.setMaxAlpha(avMaxAlpha);
        }
        return avMaxAlpha;
    }

    public double findMaxAlphas(int iAction, BeliefState bs, ArrayList<MachineState> fsc, AlphaVector[] aNext) {
        AlphaVector avAlpha = null;
        int iObservation = 0;
        double dSumValues = 0.0, dValue = 0, dProb = 0.0, dSumProbs = 0.0;
        BeliefState bsSuccessor = null;

        boolean bCache = m_pomdp.getBeliefStateFactory().isCachingBeliefStates();
        for (iObservation = 0; iObservation < m_pomdp.getObservationCount(); iObservation++) {
            dProb = bs.probabilityOGivenA(iAction, iObservation);
            dSumProbs += dProb;
            if (dProb > 0.0) {
                bsSuccessor = bs.nextBeliefState(iAction, iObservation);
                avAlpha = getMaxAlpha(bsSuccessor, fsc);
                dValue = avAlpha.dotProduct(bsSuccessor);
                dSumValues += dValue * dProb;
            } else {
                avAlpha = fsc.get(fsc.size() - 1).getVec();
            }
            aNext[iObservation] = avAlpha;

        }

        dSumValues /= dSumProbs; //in case due to rounding there is an error and probs do not exactly sum to 1
        dSumValues *= m_pomdp.getDiscountFactor();
        dSumValues += m_pomdp.immediateReward(bs, iAction);

        return dSumValues;
    }

    private AlphaVector G(int iAction, AlphaVector[] aNext) {
        AlphaVector avAlpha = null, avG = null, avSum = null, avResult = null;
        int iObservation = 0;

        for (iObservation = 0; iObservation < m_pomdp.getObservationCount(); iObservation++) {
            avAlpha = aNext[iObservation];
            avG = avAlpha.G(iAction, iObservation);
            if (avSum == null)
                avSum = avG.copy();
            else
                avSum.accumulate(avG);
        }

        avResult = avSum.addReward(iAction);
        avResult.setAction(iAction);

        avSum.release();
        return avResult;
    }

    public AlphaVector backup(BeliefState bs, ArrayList<MachineState> fsc) {
        double dValue = 0.0, dMaxValue = Double.NEGATIVE_INFINITY;
        int iAction = 0, iMaxAction = -1;
        AlphaVector[] aNext = null, aBest = null;

        Vector<AlphaVector[]> vWinners = new Vector<>();
        Vector<Integer> vWinnersActions = new Vector<>();

        for (iAction = 0; iAction < m_pomdp.getActionCount(); iAction++) {
            aNext = new AlphaVector[m_pomdp.getObservationCount()];
            dValue = findMaxAlphas(iAction, bs, fsc, aNext);

            if (dValue > dMaxValue) {
                dMaxValue = dValue;
                vWinners.clear();
                vWinnersActions.clear();
            }
            if (dValue == dMaxValue) {
                aBest = aNext;
                iMaxAction = iAction;
                vWinners.add(aBest);
                vWinnersActions.add(iMaxAction);
            }
        }

        int idx = new Random(System.nanoTime()).nextInt(vWinners.size());
        aBest = vWinners.elementAt(idx);
        iMaxAction = vWinnersActions.elementAt(idx);
        AlphaVector avMax = G(iMaxAction, aBest);
        avMax.setWitness(bs);

        return avMax;
    }

    public ArrayList<MachineState> updateMachineState(ArrayList<MachineState> fsc, ArrayList<BeliefState> pointList, double gama) {
        System.out.println("updateMachineState started!");
        ArrayList<MachineState> TODO = new ArrayList<>();
        for (int i = 0; i < pointList.size(); ++i) {
            BeliefState bsCurrent = pointList.get(i);
            AlphaVector avCurrentMax = getMaxAlpha(bsCurrent, fsc);
            AlphaVector avBackup = backup(bsCurrent, fsc);

            double dBackupValue = avBackup.dotProduct(bsCurrent);
            double dValue = avCurrentMax.dotProduct(bsCurrent);
            if (dBackupValue - dValue < 0) {
                continue;
            }

            MachineState msTemp = new MachineState();
            msTemp.setAction(avBackup.getAction());
            msTemp.setVec(avBackup);
            msTemp.getPointGroup().add(bsCurrent);
            msTemp.buildLink(fsc, m_pomdp.getObservationCount());

            ArrayList<MachineState> same = new ArrayList<>();
            for (MachineState ms : fsc) {
                if (ms.compareLink(msTemp)) {
                    same.add(ms);
                }
            }

            if (same.size() == 0) {
                fsc.add(msTemp);
                MachineState msOriginal = bsCurrent.getMs();
                msOriginal.getPointGroup().remove(bsCurrent);
                TODO.add(msOriginal);
            } else {
                for (MachineState ms : same) {
                    for (BeliefState bs : ms.getPointGroup()) {
                        msTemp.getPointGroup().add(bs);
                    }
                }


                for (MachineState ms : fsc) {
                    if (same.contains(ms)) {
                        continue;
                    }
                    HashMap<Integer, MachineState> link = ms.outwardLink;
                    for (Integer o : link.keySet()) {
                        for (MachineState msSame : same) {
                            if (link.get(o).equals(msSame)) {
                                link.put(o, msTemp);
                            }
                        }

                    }
                }

                for (int j = fsc.size() - 1; j >= 0; j--) {
                    MachineState ms = fsc.get(j);
                    if (same.contains(ms)) {
                        fsc.remove(ms);
                    }
                }
                fsc.add(msTemp);

            }

        }
        this.deleteMSwithNoPoint(TODO);
        System.out.println("updateMachine Complete!");
        System.out.println("FSC size = " + fsc.size());
        return fsc;
    }


    public double iterationCalculation(ArrayList<BeliefStateVector<BeliefState>> dbscanResult, double gama, double epsilon) {
        ArrayList<MachineState> prevMSList = new ArrayList<>();
        ArrayList<MachineState> newMSList = fsc;
        while (!convergence(prevMSList, newMSList)) {
            System.out.println("first level iteration started:");
            ArrayList<BeliefStateVector<BeliefState>> copyResult = new ArrayList<>();
            try {
                prevMSList = deepCopyMS(newMSList);
                copyResult = deepCopyDBSCANResult(dbscanResult);
            } catch (ClassNotFoundException | IOException e) {
                //e.printStackTrace();
            }

            while (!isAllMarked(copyResult)) {
                System.out.println("second level iteration started:");
                ArrayList<BeliefState> pointList = samplePoints(copyResult, epsilon);
                System.out.println("samplePoints size = " + pointList.size());
                newMSList = updateMachineState(newMSList, pointList, gama);
                System.out.println("MachineState updated, size = " + newMSList.size());
                System.out.println(pointValueAve(newMSList));
            }
            fsc = newMSList;
        }
        return pointValueAve(newMSList);
    }

    public boolean convergence(ArrayList<MachineState> fore, ArrayList<MachineState> rear) {
        if (fore.size() != rear.size()) {
            return false;
        }
//        for (MachineState ms : rear) {
//            if(!fore.contains(ms)){
//                return false;
//            }
//        }
        return true;
    }

    private double pointValueAve(ArrayList<MachineState> fsc) {
        BigDecimal sumVal = new BigDecimal(0);
        for (MachineState aFsc : fsc) {
            for (int k = 0; k < aFsc.pointGroup.size(); k++) {
                sumVal = sumVal.add(new BigDecimal(aFsc.getVec().dotProduct(aFsc.getPointGroup().get(k))));
            }
        }

        return sumVal.divide(new BigDecimal(vBeliefPoints.size()), 10, BigDecimal.ROUND_HALF_UP).doubleValue();


    }

    public ArrayList<BeliefStateVector<BeliefState>> deepCopyDBSCANResult(ArrayList<BeliefStateVector<BeliefState>> dbscanResult)
            throws IOException, ClassNotFoundException {
        ByteArrayOutputStream byteOut = new ByteArrayOutputStream();
        ObjectOutputStream out = new ObjectOutputStream(byteOut);
        out.writeObject(dbscanResult);

        ByteArrayInputStream byteIn = new ByteArrayInputStream(byteOut.toByteArray());
        ObjectInputStream in = new ObjectInputStream(byteIn);
        ArrayList<BeliefStateVector<BeliefState>> result = (ArrayList<BeliefStateVector<BeliefState>>) in.readObject();
        return result;
    }

    @SuppressWarnings("unchecked")
    public ArrayList<MachineState> deepCopyMS(ArrayList<MachineState> ms) throws IOException, ClassNotFoundException {
        ByteArrayOutputStream byteOut = new ByteArrayOutputStream();
        ObjectOutputStream out = new ObjectOutputStream(byteOut);
        out.writeObject(ms);

        ByteArrayInputStream byteIn = new ByteArrayInputStream(byteOut.toByteArray());
        ObjectInputStream in = new ObjectInputStream(byteIn);
        ArrayList<MachineState> result = (ArrayList<MachineState>) in.readObject();
        return result;
    }

}
