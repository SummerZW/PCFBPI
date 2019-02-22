package dbscan;

/**
 * Created with IntelliJ IDEA.
 * User: hanbing
 * Date: 14-2-17
 * Time: 涓嬪崍5:28
 * To change this template use File | Settings | File Templates.
 */

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.*;

import pomdp.utilities.BeliefState;
import pomdp.utilities.BeliefStateVector;

public final class Utility {
    //璁＄畻涓ょ偣涔嬮棿鐨勮窛绂�
    public static double getDistance(Point p, Point q) {
        double distance = 0;

        for (int i = 0; i < p.dim.length; i++) {
            distance += Math.abs(p.dim[i] * p.dim[i] - q.dim[i] * q.dim[i]);
        }
        return Math.sqrt(distance);
    }

    //妫�祴p鐐规槸涓嶆槸鏍稿績鐐癸紝tmpLst瀛樺偍鏍稿績鐐圭殑鐩磋揪鐐�
    public static List<Point> isKeyPoint(List<Point> lst, Point p, double e, int minp) {
        int count = 0;
        List<Point> tmpLst = new ArrayList<Point>();
        for (Iterator<Point> it = lst.iterator(); it.hasNext(); ) {
            Point q = it.next();
            if (getDistance(p, q) <= e) {
                ++count;
                if (!tmpLst.contains(q)) {
                    tmpLst.add(q);
                }
            }
        }
        if (count >= minp) {
            p.setKey(true);
            return tmpLst;
        }
        return null;
    }

    //鍚堝苟涓や釜閾捐〃锛屽墠鎻愭槸b涓殑鏍稿績鐐瑰寘鍚湪a涓�
    public static boolean mergeList(List<Point> a, List<Point> b) {
        boolean merge = false;
        if (a == null || b == null) {
            return false;
        }
        for (int index = 0; index < b.size(); ++index) {
            Point p = b.get(index);
            if (p.isKey() && a.contains(p)) {
                merge = true;
                break;
            }
        }
        if (merge) {
            for (int index = 0; index < b.size(); ++index) {
                if (!a.contains(b.get(index))) {
                    a.add(b.get(index));
                }
            }
        }
        return merge;
    }

    //鑾峰彇鏂囨湰涓殑鏍锋湰鐐归泦鍚�
    public static List<Point> getPointsList(String txtPath) throws IOException {
        int count = 0;
        List<Point> lst = new ArrayList<Point>();
        BufferedReader br = new BufferedReader(new FileReader(txtPath));
        String str = "";
        while ((str = br.readLine()) != null && str != "") {
            lst.add(new Point(str, count));
            count++;
        }
        br.close();
        return lst;
    }

    public static List<Point> getPointsListFromBeliefStateVector(BeliefStateVector<BeliefState> beliefStateVector, int stateCount) throws IOException {
        int count = 0;
        List<Point> lst = new ArrayList<Point>();
        for (BeliefState beliefState : beliefStateVector) {
            lst.add(new Point(beliefState, stateCount));
        }
        return lst;
    }

    //鏄剧ず鑱氱被鐨勭粨鏋�
    public static void display(List<List<Point>> resultList) {
        int index = 1;
        for (Iterator<List<Point>> it = resultList.iterator(); it.hasNext(); ) {
            List<Point> lst = it.next();
            if (lst.isEmpty()) {
                continue;
            }
            System.out.println("#" + index);
//            for (Iterator<Point> it1 = lst.iterator(); it1.hasNext(); ) {
//                Point p = it1.next();
//                System.out.println(p.print());
//            }
            index++;
        }
    }
}

