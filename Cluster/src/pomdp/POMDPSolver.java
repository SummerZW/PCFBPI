package pomdp;

import pomdp.algorithms.pointbased.ClusterValueIteration;
import pomdp.environments.POMDP;
import pomdp.utilities.Logger;

public class POMDPSolver {

    public static void main(String[] args) {
        String sPath = "Models/";// model路径
        String sModelName = "hallway2";// model名
        String sMethodName = "PCFBPI";// 方法名
        Logger.getInstance().setOutput(true);// 允许输出
        Logger.getInstance().setSilent(false);// 允许输出到控制台
        try {
            String sOutputDir = "logs/POMDPSolver";// 输出路径
            String sFileName = sModelName + "_" + sMethodName + ".txt";// 输出文件名
            Logger.getInstance().setOutputStream(sOutputDir, sFileName);
        } catch (Exception e) {
            System.err.println(e);
        }

        POMDP pomdp = null;
        try {
            pomdp = new POMDP();
            pomdp.load(sPath + sModelName + ".POMDP");// 载入pomdp模型,  用来代替聚类代码中的Model

            // 输出最大回报值和最小回报值
            // Logger.getInstance().logln("max is " + pomdp.getMaxR() + " min is " + pomdp.getMinR());
        } catch (Exception e) {
            Logger.getInstance().logln(e);
            e.printStackTrace();
            System.exit(0);
        }
        try {
            ClusterValueIteration iteration = new ClusterValueIteration(pomdp);//开始策略迭代
            iteration.clusterIteration(pomdp);
        } catch (Exception e) {
            Logger.getInstance().logln(e);
            e.printStackTrace();
            System.exit(0);
        }
    }
}
