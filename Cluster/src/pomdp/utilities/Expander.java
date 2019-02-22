package pomdp.utilities;

import pomdp.environments.POMDP;
import pomdp.utilities.distance.L1Distance;

import java.util.LinkedList;
import java.util.Queue;

public class Expander {

    private POMDP pomdp;
    private int LEVEL;

    public Expander(POMDP pomdp, int level) {
        this.pomdp = pomdp;
        this.LEVEL = level;
    }

    public BeliefStateVector<BeliefState> expand(double epsilon) {
        BeliefStateVector<BeliefState> vBeliefPoints = new BeliefStateVector<>();
        Queue<BeliefState> q = new LinkedList<>(); // 队列先进先出，适用于广度优先探索
        BeliefState initial = pomdp.getBeliefStateFactory().getInitialBeliefState();
        vBeliefPoints.add(initial);
        q.offer(initial);
        while (!q.isEmpty()) {
            BeliefState bs = q.poll();
            for (int iAction = 0; iAction < pomdp.getActionCount(); ++iAction) {
                for (int iObservation = 0; iObservation < pomdp.getObservationCount(); ++iObservation) {
                    BeliefState next = bs.nextBeliefState(iAction, iObservation);

                    if (next != null && !vBeliefPoints.contains(next)) {
                        next.setLevel(bs.getLevel() + 1); // 深度向下一层

                        // 判断next点与当前已探索的任意一点是否距离大于epsilon，大于则加入
                        boolean inRange = false;
                        L1Distance distancer = new L1Distance();
                        for (BeliefState beliefState : vBeliefPoints) {
                            if (distancer.distance(beliefState, next) < epsilon) {
                                inRange = true;
                                break;
                            }
                        }
                        if (!inRange) {
                            if (next.getLevel() <= LEVEL) {
                                q.offer(next);
                                vBeliefPoints.add(next);
                                System.out.println("Points number:" + vBeliefPoints.size() + " height:" + next.getLevel());
                            } else {
                                System.out.println(vBeliefPoints.size());
                                return vBeliefPoints;
                            }
                        }
                    }
                }
            }
        }
        return vBeliefPoints;
    }
}
