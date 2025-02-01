
#ifndef PQP_GETTIME_H
#define PQP_GETTIME_H

#ifdef WIN32

  #include <time.h>
  #include <sys/timeb.h>
  inline
  double 
  GetTime()
  {
    struct _timeb thistime;
    _ftime(&thistime);    
    return (thistime.time + thistime.millitm * 1e-3);
  }

#else

  #include <sys/time.h>
  inline
  double 
  GetTime()
  {
    struct timeval thistime;
    gettimeofday(&thistime, 0);    
    return (thistime.tv_sec + thistime.tv_usec * 1e-6);
  }

#endif

#endif
