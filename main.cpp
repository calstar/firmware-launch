#include "msg_downlink_generated.h"
#include "msg_uplink_generated.h"
#include "msg_fc_update_generated.h"

#if !defined(bb) && !defined(fc) && !defined(gs) && !defined(tpc)
#error "Must define one of: bb, fc, gs, tpc"
#endif

#ifdef bb
    #include "bb/bb.h"
#endif
#ifdef fc
    #include "fc/fc.h"
#endif
#ifdef gs
    #include "gs/gs.h"
#endif
#ifdef tpc
    #include "tpc/tpc.h"
#endif
