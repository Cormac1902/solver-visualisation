#ifdef __cplusplus
extern "C" {
#endif
/************************************/
/* Standard includes                */
/************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/types.h>
#include <limits.h>
#include <signal.h>

/************************************/
/* Compilation flags                */
/************************************/

/********************************************************************/
/* Following tests set exactly one of the following flags to 1:     */
/*    BSD:   BSD Unix                                               */
/*    OSX:   Apple OS X                                           */
/*    LINUX: Linux Unix                                           */
/*    WINDOWS: Windows and DOS. Linking requires -l Winmm.lib       */
/*    POSIX: Other POSIX OS                                         */
/* Platform dependent differences:                                  */
/*    -arc4 only available on BSD and OSX. Option uses slower but   */
/*        stronger random number generation arc4random and          */
/*        arc4random_uniform                                        */
/*    WINDOWS and POSIX use rand() instead of random()              */
/*    Clock ticks per second determined by sysconf(_SC_CLK_TCK)     */
/*        for BSD, OSX, and LINUX                                   */
/*    Clock ticks per second fixed at 1000 for Windows              */
/*    Clock ticks per second fixed at 1 for POSIX                   */
/********************************************************************/

#if __FreeBSD__ || __NetBSD__ || __OpenBSD__ || __bsdi__ || _SYSTYPE_BSD
#define BSD 1
#elif __APPLE__ && __MACH__
#define OSX 1
#elif __unix__ || __unix || unix || __gnu_linux__ || linux || __linux
#define LINUX 1
#elif _WIN64 || _WIN23 || _WIN16 || __MSDOS__ || MSDOS || _MSDOS || __DOS__
#define NT 1
#else
#define POSIX 1
#endif

/* Define BIGINT to be the type used for the "cutoff" variable.
   Under gcc "long long int" gives a 64 bit integer.
   Under Windows __int64 gives a 64 bit integer.
   No way under POSIX to guarantee long int is 64 bits.
   Program will still function using a 32-bit value, but it
   limit clauseSize of cutoffs that can be specified. */

#if BSD || OSX || LINUX
#define BIGINT long long int
#define BIGFORMAT "lli"
#elif WINDOWS
#define BIGINT __int64
#define BIGFORMAT "I64d"
#elif POSIX
#define BIGINT long int
#define BIGFORMAT "li"
#endif


#if BSD || OSX || LINUX

#include <sys/times.h>
#include <sys/time.h>
#include <unistd.h>

#elif WINDOWS
#include <time.h>
#include <windows.h>
#include <mmsystem.h>
#elif POSIX
#include <sys/time.h>
#endif

#if POSIX || WINDOWS
#define random() rand()
#define srandom(seed) srand(seed)
#endif

/************************************/
/* Constant parameters              */
/************************************/

#define MAXFILENAME 2048
#define TRUE 1
#define FALSE 0
#define BIG 1000000000           /* a number bigger that the possible number of violated clauses */

#define HISTMAX 64        /* length of histogram of tail */
#define MAXATTEMPT 10           /* max number of times to attempt to find a non-tabu variable to flip */
#define denominator 100000       /* denominator used in fractions to represent probabilities */
#define ONE_PERCENT 1000         /* ONE_PERCENT / denominator = 0.01 */

/* #define DEBUG */

/************************************/
/* Main data structures             */
/************************************/

/* Atoms start at 1 */
/* Not a is recorded as -1 * a */
/* One dimensional arrays are statically allocated. */
/* Two dimensional arrays are dynamically allocated in */
/* the second dimension only.  */

int numatom;                    /* number of atoms */
int numclause;                  /* number of clauses */
int numliterals;                /* number of instances of literals across all clauses */

int numfalse;            /* number of falseClauses clauses */
int numfreebie;                 /* number of freebies */


/* Data structures for clauses */

int **clause_array;            /* clauses to be satisfied */
/* indexed as clause_array[clause_num][literal_num] */
int *clauseSize;            /* length of each clause_array */
int *falseClauses;            /* clauses which are falseClauses */
int *lowfalse;                 /* clauses that are falseClauses in the best solution found so far */
int *wherefalse;        /* where each clause_array is listed in falseClauses */
int *numtruelit;        /* number of true literals in each clause_array */
int longestclause;

/* Data structures for atoms: arrays of size numatom+1 indexed by atom */

int *atom;            /* value of each atom */
int *lowatom;               /* value of best state found so far */
int *solution;              /* value of solution */
BIGINT *changed;        /* step at which atom was last flipped */
int *breakcount;        /* number of clauses that become unsat if var if flipped */
int *makecount;                /* number of clauses that become sat if var if flipped */
int *freebielist;           /* list of freebies */
int *wherefreebie;          /* where atom appears in freebies list, -1 if it does not appear */

/* Data structures literals: arrays of clauseSize 2*numatom+1, indexed by literal+numatom */

int **occurrence;                /* where each literal occurs, clauseSize 2*numatom+1            */
/* indexed as occurrence[literal+numatom][occurrence_num] */

int *numoccurrence;        /* number of times each literal occurs, clauseSize 2*numatom+1  */
/* indexed as numoccurrence[literal+numatom]              */

/* Data structures for lists of clauses used in heuristics */

int *best;
int *besttabu;
int *any;

/************************************/
/* Global flags and parameters      */
/************************************/

/* Options */

FILE *cnfStream;
//int status_flag;		/* value returned from main procedure */
int abort_flag;
int heuristic;        /* heuristic to be used */

int numerator;            /* make random flip with numerator/denominator frequency */
/*int arc4;
double walk_probability;
int plus_flag; *//* for novelty heuristics */
int tabu_length;        /* length of tabu list */
BIGINT numflip;        /* number of changes so far */
/*int numrun;
BIGINT cutoff;
BIGINT base_cutoff;
int target;
int numtry;			*//* total attempts at solutions *//*
int numsol;	                *//* stop after this many tries succeeds *//*
int superlinear;
int makeflag;		*//* set to true by heuristics that require the make values to be calculated *//*
char initfile[MAXFILENAME];
int initoptions;

int nofreebie;
int maxfreebie;
int freebienoise;
double freebienoise_prob;

int alternate_greedy;
int alternate_walk;
int alternate_greedy_state;
int alternate_run_remaining;

int adaptive;         *//* update noise level adaptively during search */
int stagnation_timer;         /* number of remaining flips until stagnation check is performed */
int last_adaptive_objective;  /* number of unsat clauses last time noise was adaptively updated */
double adaptive_phi;
double adaptive_theta;

/* Random seed */

unsigned int seed;  /* Sometimes defined as an unsigned long int */

#if BSD || LINUX || OSX
struct timeval tv;
struct timezone tzp;
long ticks_per_second;
#elif NT
DWORD win_time;     /* elapsed time in ms, since windows boot up */
#endif

/* Histogram of tail */

BIGINT tailhist[HISTMAX];    /* histogram of num unsat in tail of run */
long histtotal;
//int tail;
int tail_start_flip;
//int undo_age;
BIGINT undo_count;

/* Printing options */

/*int printonlysol;
int printsolcnf;
int printfalse;
int printlow;
int printhist;
int printtrace;
int trace_assign;
char outfile[MAXFILENAME];*/

/* Statistics */

double expertime;
//BIGINT flips_this_solution;
int lowbad;                /* lowest number of bad clauses during try */
/*BIGINT totalflip;		*//* total number of flips in all tries so far *//*
BIGINT totalsuccessflip;	*//* total number of flips in all tries which succeeded so far *//*
int numsuccesstry;		*//* total found solutions */
BIGINT x;
//BIGINT integer_sum_x;
//double sum_x;
//double sum_x_squared;
double mean_x;
double second_moment_x;
double variance_x;
double std_dev_x;
double std_error_mean_x;
double seconds_per_flip;
int r;
/*int sum_r;
double sum_r_squared;*/
double mean_r;
double variance_r;
double std_dev_r;
double std_error_mean_r;
double avgfalse;
double sumfalse;
double sumfalse_squared;
double second_moment_avgfalse, variance_avgfalse, std_dev_avgfalse, ratio_avgfalse;
double f;
double sample_size;
/*double sum_avgfalse;
double sum_std_dev_avgfalse;*/
double mean_avgfalse;
double mean_std_dev_avgfalse;
//int number_sampled_runs;
double ratio_mean_avgfalse;
/*double suc_sum_avgfalse;
double suc_sum_std_dev_avgfalse;*/
double suc_mean_avgfalse;
double suc_mean_std_dev_avgfalse;
//int suc_number_sampled_runs;
double suc_ratio_mean_avgfalse;
/*double nonsuc_sum_avgfalse;
double nonsuc_sum_std_dev_avgfalse;*/
double nonsuc_mean_avgfalse;
double nonsuc_mean_std_dev_avgfalse;
//int nonsuc_number_sampled_runs;
double nonsuc_ratio_mean_avgfalse;

void *context;
void *variable_activity_sender;

char *int_to_send;

/* Hamming calculations */

/*char hamming_target_file[MAXFILENAME];
char hamming_data_file[MAXFILENAME];*/
int hamming_sample_freq;
//int hamming_flag;
int hamming_distance;
int *hamming_target;
FILE *hamming_fp;

/**************************************/
/* Inline utility functions           */
/**************************************/

static inline int ABS(int x) { return x < 0 ? -x : x; }

static inline int LENGTH(int x) {
    if (x == 0) return 1;

    return (int) floor(log10(abs(x))) + 1;
}

static inline int RANDMOD(int x) {
#if OSX || BSD
    return x > 1 ? (arc4 ? arc4random_uniform((uint32_t) x) : random()%x) : 0;
#else
    return x > 1 ? random() % x : 0;
#endif
}


static inline int MAX(int x, int y) { return x > y ? x : y; }

static inline int MIN(int x, int y) { return x < y ? x : y; }

static inline int onfreebielist(int v) { return wherefreebie[v] != -1; }

static inline void addtofreebielist(int v) {
    freebielist[numfreebie] = v;
    wherefreebie[v] = numfreebie++;
}

static inline void removefromfreebielist(int v) {
    int swapv;
    int wherev;

    if (numfreebie < 1 || wherefreebie[v] < 0) {
        fprintf(stderr, "Freebie list error!\n");
        exit(-1);
    }
    numfreebie--;
    wherev = wherefreebie[v];
    wherefreebie[v] = -1;
    if (wherev == numfreebie) return;
    swapv = freebielist[numfreebie];
    freebielist[wherev] = swapv;
    wherefreebie[swapv] = wherev;
}

static inline char *INT_STRING(int x) {
    sprintf(int_to_send, "%d", x);
    return int_to_send;
}

static inline char *PORT_STRING(int port) {
    char *prefix = "tcp://localhost:";
    char *str = (char*) malloc(strlen(prefix) + LENGTH(port));
    sprintf(str, "%s", prefix);
    sprintf(str + strlen(str), "%d", port);
    return str;
}

/************************************/
/* Forward declarations             */
/************************************/

void parse_parameters(int argc, char *argv[]);

void print_parameters();

int pickrandom(void);

int pickbest(void);

int picktabu(void);

int picknovelty(void);

int pickrnovelty(void);

int pickalternate(void);

int pickbigflip(void);

int pickgsat(void);

enum heuristics {
    RANDOM, BEST, TABU, NOVELTY, RNOVELTY,
    ALTERNATE, BIGFLIP, GSAT
};

static int (*pickcode[])(void) =
        {pickrandom, pickbest, picktabu,
         picknovelty, pickrnovelty,
         pickalternate, pickbigflip, pickgsat};

double elapsed_seconds(void);

int countunsat(void);

void scanone(int argc, char *argv[], int i, int *varptr);

void scanonell(int argc, char *argv[], int i, BIGINT *varptr);

void scanoned(int argc, char *argv[], int i, double *varptr);

void init(void);

void initprob(unsigned int longest_cl, int **clauses, int num_atom, int num_clause);

void flipatom(int toflip);

void print_false_clauses(int lowbad);

void save_false_clauses(int lowbad);

void print_low_assign(int lowbad);

void save_low_assign(void);

void save_solution(void);

void print_current_assign(void);

void handle_interrupt(int sig);

long super(int i);

void print_sol_file(char *filename);

void print_statistics_header(void);

void initialize_statistics(void);

void update_statistics_start_try(void);

void print_statistics_start_flip(void);

void update_and_print_statistics_end_try(void);

void update_statistics_end_flip(void);

void print_statistics_final(void);

void print_sol_cnf(void);

void read_hamming_file(void);

void open_hamming_data(void);

int calc_hamming_dist(int atom[], int hamming_target[], int numatom);

int solve_walksat(unsigned int longest_cl, int **clauses, int num_atom, int num_clause);

void send_variable_activity(int var);

#ifdef __cplusplus
}
#endif