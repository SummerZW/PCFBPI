package pomdp.utilities;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.HashMap;

public class MachineState implements Serializable {

    private static final long serialVersionUID = 1L;
    private int action;
    private AlphaVector vec;
    public HashMap outwardLink = new HashMap<Integer, MachineState>();
    public BeliefStateVector<BeliefState> pointGroup = new BeliefStateVector<>();
    public BeliefStateVector<BeliefState> samplePoints = new BeliefStateVector<>();

    public int getAction() {
        return action;
    }

    public BeliefStateVector<BeliefState> getSamplePoints() {
        return samplePoints;
    }

    public BeliefStateVector<BeliefState> getPointGroup() {
        return pointGroup;
    }

    public void setAction(int action) {
        this.action = action;
    }

    public AlphaVector getVec() {
        return vec;
    }

    public void setVec(AlphaVector vec) {
        this.vec = vec;
    }

    public HashMap getOutwardLink() {
        return outwardLink;
    }

    public void buildLink(ArrayList<MachineState> fsc, int o) {
        BeliefState bs = this.getPointGroup().get(0);
        for (int i = 0; i < o; i++) {
            BeliefState bsNext = bs.nextBeliefState(this.action, i);
            if (bsNext == null) {
                continue;
            }
            MachineState msNext = bsNext.getMs();
            if (msNext == null) {
                continue;
            }
            this.outwardLink.put(i, msNext);
        }
    }

    public boolean compareLink(MachineState ms) {
        HashMap<Integer, MachineState> link = ms.getOutwardLink();
        for (int i = 0; i < link.size(); i++) {
            if (this.outwardLink.get(i) == null) {
                if (link.get(i) != null) {
                    return false;
                }
            }
            if(link.get(i)==null){
                return false;
            }
            if (!this.outwardLink.get(i).equals(link.get(i))) {
                return false;
            }
        }
        return true;
    }
}
