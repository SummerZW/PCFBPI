package pomdp.algorithms.pointbased;


import pomdp.algorithms.ValueIteration;
import pomdp.environments.POMDP;
import pomdp.utilities.*;

import java.io.*;
import java.util.ArrayList;
import java.util.Iterator;

public class ClusterValueIteration extends ValueIteration {

    protected Iterator<BeliefState> m_itCurrentIterationPoints;
    protected boolean m_bSingleValueFunction = true;
    protected boolean m_bRandomizedActions;
    protected double m_dFilteredADR = 0.0;

    ArrayList<MachineState> fsc = new ArrayList<>(); // 后来加的
    double gamma = 0.95;
    public static final double EPSILON = 0.5;

    public ClusterValueIteration(POMDP pomdp) {
        super(pomdp);

        m_itCurrentIterationPoints = null;
        m_bRandomizedActions = true;
    }

    public ClusterValueIteration(POMDP pomdp, boolean bRandomizedActionExpansion) {
        super(pomdp);

        m_itCurrentIterationPoints = null;
        m_bRandomizedActions = bRandomizedActionExpansion;
    }


    public int getAction(BeliefState bsCurrent) {
        return m_vValueFunction.getBestAction(bsCurrent);
    }


    public void clusterIteration(POMDP pomdp) throws IOException, ClassNotFoundException {
//        BeliefState b0 = pomdp.getBeliefStateFactory().getInitialBeliefState();
//        BeliefState b1 = this.deepCopyBS(b0);
//        BeliefStateComparator bsc = new BeliefStateComparator(0.00000000001);
//        System.out.println("result"+ bsc.compare(b0, b1));
//        System.exit(0);

        // 点集扩张
        Expander expander = new Expander(pomdp,4);
        BeliefStateVector<BeliefState> vBeliefPoints = expander.expand(EPSILON);

        // DBSCAN ( minPts = 总点数/150, epsilon = e * 600,000,000 / 总点数 )
        DBSCAN dbscan = new DBSCAN(10);
        ArrayList<BeliefStateVector<BeliefState>> clusterResult = dbscan.DBSCAN(vBeliefPoints, 0.55);

        // 初始化FSC
        FSCSolver fscSolver = new FSCSolver(vBeliefPoints, pomdp);
        fscSolver.initialFSC(gamma);

        // 迭代计算
        double result = fscSolver.iterationCalculation(clusterResult, gamma, 0.0001);
        System.out.println("result: " + result);
    }

    @Override
    public void clusterIteration() {

    }

    public BeliefState deepCopyBS(BeliefState bs) throws IOException, ClassNotFoundException {
        ByteArrayOutputStream byteOut = new ByteArrayOutputStream();
        ObjectOutputStream out = new ObjectOutputStream(byteOut);
        out.writeObject(bs);

        ByteArrayInputStream byteIn = new ByteArrayInputStream(byteOut.toByteArray());
        ObjectInputStream in = new ObjectInputStream(byteIn);
        BeliefState result = (BeliefState) in.readObject();
        return result;
    }
}
