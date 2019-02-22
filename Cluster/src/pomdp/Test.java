package pomdp;

import pomdp.utilities.FSCSolver;
import pomdp.utilities.MachineState;

import java.io.IOException;
import java.util.ArrayList;

public class Test {
    public static void main(String[] args) throws IOException, ClassNotFoundException {
        Test t = new Test();
        t.func2();
    }

    private class A {
        public int a;

        public A(int a) {
            this.a = a;
        }
    }

    public void func1() throws IOException, ClassNotFoundException {
        MachineState m1 = new MachineState();
        MachineState m2 = new MachineState();
        ArrayList<MachineState> ms1 = new ArrayList<>();
        ms1.add(m1);
        ms1.add(m2);
        FSCSolver f = new FSCSolver();
        ArrayList<MachineState> ms2 = f.deepCopyMS(ms1);
        System.out.println(f.convergence(ms1, ms2));
        System.out.println();
    }

    public void func2() {
        ArrayList<A> as = new ArrayList<>();
        for (int i = 0; i < 10; i++) {
            as.add(new A(i));
        }
//        Iterator<Integer> iterator = ms.iterator();
//        while (iterator.hasNext()) {
//            int i = iterator.next().intValue();
//            if (i % 2 == 0) {
//                iterator.remove();
//            }
//        }
        for (int i = as.size() - 1; i >= 0; i--) {
            A a = as.get(i);
            if (a.a % 2 == 0) {
                as.remove(a);
            }
        }
        System.out.println(as.size());
    }

    public void func3() throws IOException, ClassNotFoundException {
    }
}
