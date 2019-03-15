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
