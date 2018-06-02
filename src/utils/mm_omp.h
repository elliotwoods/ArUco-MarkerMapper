//Header to link agains omp or not
#ifndef _MM_OMP_
#define _MM_OMP_

#ifdef USE_OMP
#include <omp.h>
#else
inline int omp_get_max_threads(){return 1;}
inline int omp_get_thread_num(){return 0;}
#endif

#endif
