/*
 * Created on May 5, 2005
 *
 * To change the template for this generated file go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
package pomdp.algorithms;

import pomdp.utilities.BeliefState;


/**
 * @author shanigu
 * <p>
 * To change the template for this generated type comment go to
 * Window&gt;Preferences&gt;Java&gt;Code Generation&gt;Code and Comments
 */
public abstract class PolicyStrategy/*  implements Serializable*/ {
    protected boolean m_bExploring;
    protected boolean m_bStationary;


    public PolicyStrategy() {
        m_bExploring = true;
    }

    /**
     * 根据信念状态返回动作
     * @param bsCurrent
     * @return
     */
    public abstract int getAction(BeliefState bsCurrent);


}
