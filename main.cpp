#include "mbed.h"
#include "msg_downlink_generated.h"
#include "msg_uplink_generated.h"
#include "msg_fc_update_generated.h"

#include "bb/bb.h"
#include "fc/fc.h"
#include "gs/gs.h"
#include "tpc/tpc.h"

#if defined(bb) || defined(fc) || defined(gs) || defined(tpc)

int main() {
    return run();
}

#else
#error "Must define one of: bb, fc, gs, tpc"
#endif