package pomdp.utilities;

import pomdp.utilities.distance.Kmeans;
import pomdp.utilities.distance.L1Distance;

import java.util.ArrayList;

public class Explorer {

    /**
     * 根据聚类点集计算探索点集
     * 每个聚类计算cover，每个cover取其距中心最近的点
     * 所有cover集合取并集
     */
    public BeliefStateVector<BeliefState> explore(ArrayList<BeliefStateVector<BeliefState>> bs) {
        // 每个BeliefStateVector对应一个calCoverThread
        ArrayList<calCoverThread> calCoverThreads = new ArrayList<>();
        ArrayList<ArrayList<BeliefState>> covers = new ArrayList<>();
        for (BeliefStateVector<BeliefState> bsv : bs) {
            calCoverThread calCoverThread = new calCoverThread(bsv, 0.5);
            calCoverThreads.add(calCoverThread);
        }
        // 多线程并发
        for (calCoverThread cct : calCoverThreads) {
            cct.start();
        }
        // 是否全部完成
        boolean done = false;
        while (!done) {
            // 尚未全部完成时，对所有线程轮询获取结果
            for (calCoverThread cct : calCoverThreads) {
                ArrayList<BeliefState> cover = cct.getCover();
                // 如果拿到了结果，加入covers
                if (cover != null) {
                    covers.add(cover);
                    // 如果covers的大小等于bs的大小，说明所有线程都已返回结果，可以结束
                    done = covers.size() == bs.size() ? true : false;
                }
            }
        }
        // 将所有covers中的信念点加入到最终结果集合中
        BeliefStateVector<BeliefState> result = new BeliefStateVector<>();
        for (ArrayList<BeliefState> cover : covers) {
            Logger.getInstance().logln(cover.size());
            for (BeliefState b : cover) {
                result.add(b);
            }
        }
        Logger.getInstance().logln(result.size());
        return result;
    }

    /**
     * 计算cover线程
     */
    private class calCoverThread extends Thread {

        private BeliefStateVector<BeliefState> bsv; // 需要计算cover的信念点集

        private double radius; // cover半径

        private ArrayList<BeliefStateVector<BeliefState>> result; // cover结果

        private boolean isDone; // 是否计算完成

        private boolean hasGet; // 是否已经获取过结果

        public calCoverThread(BeliefStateVector<BeliefState> bsv, double radius) {
            this.bsv = bsv;
            this.radius = radius;
            this.result = new ArrayList<>();
            this.isDone = false;
            this.hasGet = false;
        }

        public void run() {
            // C={c1,c2,...,cn} ci={bi}
            ArrayList<BeliefStateVector<BeliefState>> absv = new ArrayList<>();
            for (BeliefState bs : this.bsv) {
                BeliefStateVector<BeliefState> b = new BeliefStateVector();
                b.add(bs);
                absv.add(b);
            }
            int iteration = 0;
            while (true) {
                // 计算d(cover)
                double[] result = dCover(absv);
                iteration++;
                Logger.getInstance().logln(iteration + ":" + result[0]);
                // d(cover) <= 2*半径
                if (result[0] <= 2 * radius) {
                    // d(cover)对应的ci,cj
                    BeliefStateVector<BeliefState> bsvi = absv.get((int) result[1]);
                    BeliefStateVector<BeliefState> bsvj = absv.get((int) result[2]);
                    BeliefStateVector<BeliefState> bsvU = new BeliefStateVector<>();
                    for (BeliefState b : bsvi) {
                        bsvU.add(b);
                    }
                    for (BeliefState b : bsvj) {
                        bsvU.add(b);
                    }
                    // d(cover)U{ci,cj}\{{ci},{cj}}
                    absv.remove(bsvi);
                    absv.remove(bsvj);
                    absv.add(bsvU);
                } else {
                    // 计算结束后，已经完成设置为true，设置result
                    this.isDone = true;
                    this.result = absv;
                    break;
                }
            }
        }

        /**
         * 计算d(cover)
         */
        private double[] dCover(ArrayList<BeliefStateVector<BeliefState>> absv) {
            double minDistance = Double.MAX_VALUE;
            int minI = -1;
            int minJ = -1;
            for (int i = 0; i < absv.size() - 1; i++) {
                for (int j = i + 1; j < absv.size(); j++) {
                    double distance = this.dCC(absv.get(i), absv.get(j));
                    if (distance < minDistance) {
                        minDistance = distance;
                        minI = i;
                        minJ = j;
                    }
                }
            }
            return new double[]{minDistance, minI, minJ};
        }

        /**
         * 计算d(c1,c2)
         */
        private double dCC(BeliefStateVector<BeliefState> c1, BeliefStateVector<BeliefState> c2) {
            double maxDistance = Double.MIN_VALUE;
            L1Distance l1Distance = new L1Distance();
            for (BeliefState b1 : c1) {
                for (BeliefState b2 : c2) {
                    double distance = l1Distance.distance(b1, b2);
                    maxDistance = distance > maxDistance ? distance : maxDistance;
                }
            }
            return maxDistance;
        }

        /**
         * 获取cover
         */
        public ArrayList<BeliefState> getCover() {
            // 如果未完成
            if (!isDone) {
                return null;
            } else if (!hasGet) { // 完成，还未获取结果
                ArrayList<BeliefState> cover = new ArrayList<>();
                for (BeliefStateVector<BeliefState> bsv : this.result) {
                    BeliefState centroid = Kmeans.calCentroid(bsv);
                    cover.add(bsv.get((int) Kmeans.mostNear(centroid, bsv.toArrayList())[0]));
                }
                hasGet = true;
                return cover;
            } else { // 完成，已经获取结果
                return null;
            }

        }

    }
}
