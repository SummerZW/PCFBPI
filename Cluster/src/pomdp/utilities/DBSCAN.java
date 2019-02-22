package pomdp.utilities;

import pomdp.utilities.distance.L1Distance;

import java.util.ArrayList;
import java.util.Collection;

public class DBSCAN {

    private final int minPts;
    private static final L1Distance l1Distance = new L1Distance();

    public DBSCAN(final int minPts) {
        this.minPts = minPts;
    }

    /**
     * DBSCAN算法
     * @param vBeliefPoints 信念点集
     * @param epsilon 半径
     * @return ArrayList中的每一个Vector代表一个聚类
     */
    public ArrayList<BeliefStateVector<BeliefState>> DBSCAN(BeliefStateVector<BeliefState> vBeliefPoints, double epsilon) {
        int process = 0;
        ArrayList<Cluster> clusterList = new ArrayList<>();
        // find key points
        for (BeliefState beliefState : vBeliefPoints) {
            BeliefStateVector<BeliefState> tmpLst = this.getPointsAround(vBeliefPoints, beliefState, epsilon, minPts);
            if (tmpLst.size() > minPts) {
                // meet minimum requirement as a keyPoint, then convert it to Cluster Object
                Cluster cluster = new Cluster(beliefState, tmpLst);
                clusterList.add(cluster);
            }
        }

        // merge lists
        int size = clusterList.size();
        for (int i = 0; i < size; ++i) {
            for (int j = 0; j < size; ++j) {
                if (i != j) {
                    if (clusterList.get(i).isMergeable(clusterList.get(j), epsilon)) {
                        clusterList.get(i).merge(clusterList.get(j));
                        clusterList.get(j).clear();
                    }
                }
            }
        }

        // organize to result format
        ArrayList<BeliefStateVector<BeliefState>> result
                = new ArrayList<BeliefStateVector<BeliefState>>();
        for (Cluster cluster : clusterList) {
            if (!cluster.getCluster().isEmpty()) {
                BeliefStateVector<BeliefState> tmpList = new BeliefStateVector<>();
                tmpList.addAll(cluster.getCluster());
                result.add(tmpList);
            }
        }

        // organize unclustered points
        BeliefStateVector<BeliefState> unclusteredPoint = new BeliefStateVector<BeliefState>(vBeliefPoints);
        for (Cluster cluster : clusterList) {
            unclusteredPoint.removeAll(cluster.getCluster());
            unclusteredPoint.removeAll(cluster.getKeyPoints());
        }
        result.add(unclusteredPoint);

        System.out.println("cluser size is " + result.size());
//        System.exit(0);
        return result;
    }

    /**
     * 获得信念点集中某一信念点周围的信念点
     * @param vBeliefPoints 整体信念点集
     * @param p 某一信念点
     * @param e 半径
     */
    private BeliefStateVector<BeliefState> getPointsAround(BeliefStateVector<BeliefState> vBeliefPoints, BeliefState p, double e, int minp) {
        BeliefStateVector<BeliefState> tmpLst = new BeliefStateVector<BeliefState>();
        for (BeliefState beliefState : vBeliefPoints) {
            if (l1Distance.distance(beliefState, p) <= e) {
                tmpLst.add(beliefState);
            }
        }
        return tmpLst;
    }

    private class Cluster {
        BeliefStateVector<BeliefState> keyPoints;
        BeliefStateVector<BeliefState> cluster;

        private Cluster() {
            keyPoints = new BeliefStateVector<>();
            cluster = new BeliefStateVector<>();
        }

        private Cluster(BeliefState keyPoint, Collection<BeliefState> otherPoints) {
            keyPoints = new BeliefStateVector<>();
            cluster = new BeliefStateVector<>();
            this.addKeyPoint(keyPoint);
            this.addPointsToCluster(otherPoints);
        }

        public BeliefStateVector<BeliefState> getKeyPoints() {
            return keyPoints;
        }

        public BeliefStateVector<BeliefState> getCluster() {
            return cluster;
        }

        private void addKeyPoint(BeliefState keyPoint) {
            if (!this.keyPoints.contains(keyPoint)) {
                this.keyPoints.add(keyPoint);
            }
        }

        private void addPointToCluster(BeliefState newPoint) {
            if (!this.cluster.contains(newPoint) && !this.keyPoints.contains(newPoint)) {
                this.cluster.add(newPoint);
            }
        }

        private void addPointsToCluster(Collection<BeliefState> newPoints) {
            for (BeliefState newPoint : newPoints) {
                this.addPointToCluster(newPoint);
            }
        }

        private void merge(Cluster anotherCluster) {
            this.keyPoints.removeAll(anotherCluster.keyPoints);
            this.keyPoints.addAll(anotherCluster.keyPoints);
            this.cluster.removeAll(anotherCluster.cluster);
            this.cluster.addAll(anotherCluster.cluster);
        }

        private boolean isMergeable(Cluster anotherCluster, double epsilon) {
            for (BeliefState p : this.keyPoints) {
                for (BeliefState q : anotherCluster.keyPoints) {
                    if (DBSCAN.l1Distance.distance(p, q) <= epsilon) {
                        return true;
                    }
                }
            }
            return false;
        }

        public void clear() {
            this.keyPoints.clear();
            this.cluster.clear();
        }
    }
}

