package org.firstinspires.ftc.teamcode.opmodes.tuning;

import java.util.List;

final class SweepSelector {
    private SweepSelector() {}

    static int choose(List<Boolean> passed, List<Double> settleTime) {
        int best = -1;
        for (int i = 0; i < passed.size(); i++) {
            if (!passed.get(i)) continue;
            if (best < 0 || settleTime.get(i) < settleTime.get(best)) best = i;
        }
        if (best < 0) return -1;

        boolean atEdge = best + 1 < passed.size() && !passed.get(best + 1);
        if (atEdge && best - 1 >= 0 && passed.get(best - 1)) return best - 1;
        return best;
    }
}
