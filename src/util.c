/*
 * util.c
 * 
 * Various utility functions.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <assert.h>
#include <math.h>
#include "net_util.h"
#include "util.h"


void print_help()
{
    printf("\n");
    printf("USAGE:\n\t%s [OPTIONS] ACTION\n",config.program_name);
    printf("OPTIONS\n");
    printf("\t-h,      --help          display this help screen\n");
    printf("\t-v,      --version       display program version\n");
    printf("\t-c file, --config file   use configuration file 'file'\n");
    printf("\t-f,      --forceload     force startup of %s\n",config.program_name);
    printf("\t                         PID file exists\n");
    printf("ACTIONS\n");
    printf("\tstart                    start %s\n",config.program_name);
    printf("\tshutdown                 shutdown %s\n",config.program_name);
    printf("\n");
}

int read_config()
{
    char *line=NULL;
    size_t linecap=0;
    ssize_t linelen;
    char val[256];
    char var[256];
    int i;
    
    FILE *f;
    if ((f=fopen(config.config_file, "r"))==NULL) {
        fprintf(stderr, "Could not open config file %s\n",config.config_file);
        return -1;
    }
    
    while ((linelen = getline(&line, &linecap, f)) > 0) {
        if (linelen > 1) {
            if (sscanf(line, "#%256s",val))
                continue;
            
            if (!sscanf(line, "%256s = %256s",var,val)) {
                fprintf(stderr, "Could not parse config file %s\n",config.config_file);
                return -1;
            }
            
            if ((var[0] && !val[0]) || (!var[0] && val[0])) {
                fprintf(stderr, "Could not parse config file %s\n",config.config_file);
                return -1;
            }
            
            if (!var[0] && !val[0])
                continue;
            
            for (i=0; i<config.num_vars; i++) {
                if (!strcmp(var, config.var_name[i])) {
                    switch (config.var_type[i]) {
                        case 's':
                            strcpy((char*)(config.var_ptr[i]), val);
                            break;
                        case 'n':
                            *(int*)config.var_ptr[i] = atoi(val);
                            break;
                        case 'f':
                            *(double*)config.var_ptr[i] = atof(val);
                            break;
                        default:
                            break;
                    }
                }
            }
        }
    }
    
    free(line);
    fclose(f);
    return 0;
}

/*
 * Returns the current UNIX time in seconds.
 */
double
getTime(void){
  timeval_t time;
  gettimeofday(&time, NULL);
  return(time.tv_sec+ time.tv_usec*1.e-6);
}

// portable clock_gettime
int clock_gettime(int dummy, struct timespec* t) {
  struct timeval now;
  int rv = gettimeofday(&now, NULL);
  if (rv) 
    return rv;
  t->tv_sec  = now.tv_sec;
  t->tv_nsec = now.tv_usec * 1000;
  return 0;
}


