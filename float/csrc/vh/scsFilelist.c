#include "stdio.h"
#ifdef __cplusplus
extern "C" {
#endif
extern char at_least_one_object_file;
extern void *kernel_scs_file_ht_new(const void *, int);
extern int kernel_scs_file_ht_get(void *, const char *, int *);
extern int  strcmp(const char*, const char*);
  typedef struct {
    char* dFileName;
  } lPkgFileInfoStruct;

  typedef struct {
    char* dFileName;
    char* dRealFileName;
    long dFileOffset;
    unsigned long dFileSize;
    int dFileModTime;
    unsigned int simFlag;
  } lFileInfoStruct;

static int lNumOfScsFiles;
  static lFileInfoStruct lFInfoArr[] = {
  {"synopsys_sim.setup_0", "./synopsys_sim.setup", 7043, 244, 1744349256, 0},
  {"synopsys_sim.setup_1", "/afs/ece.cmu.edu/support/synopsys/synopsys.release/vcs-mx_vK-2015.09/bin/synopsys_sim.setup", 7287, 5552, 1440562118, 0},
  {"./work/64/FPOINT2_MULTI.sim", "", 0, 7043, 0, 1},
  {"linux64/packages/IEEE/lib/64/STD_LOGIC_SIGNED__.sim", "", 12839, 89862, 0, 0},
  {"linux64/packages/IEEE/lib/64/STD_LOGIC_SIGNED.sim", "", 102701, 41741, 0, 0},
  {"./work/64/FPOINT2_MULTI.mra", "", 144442, 8, 0, 0},
  {"./work/64/FPOINT2_MULTI.mra", "", 144450, 8, 0, 0},
  {"./work/64/FPOINT2_MULTI__FPMULTI.sim", "", 144458, 28910, 0, 1},
  {"./work/64/TOPLEVEL_CONFIGURATION_DEFAULT_FPOINT2_MULTI_FPMULTI.sim", "", 173368, 29540, 0, 0},
  {"./work/64/FPOINT2_MULTI_DATAPATH.sim", "", 202908, 6812, 0, 1},
  {"./work/64/FPOINT2_MULTI_DATAPATH__FP_DATA_PATH.sim", "", 209720, 41517, 0, 1},
  {"./work/64/FPDIV.sim", "", 251237, 4835, 0, 1},
  {"./work/64/FPDIV__BEH.sim", "", 256072, 61408, 0, 1},
  {"./work/64/FPOINT2_MULTI_DSPBA_LIBRARY_PACKAGE.sim", "", 317480, 2472, 0, 1},
  {"./altera_mf_lib/64/ALTERA_MF_COMPONENTS.sim", "", 319952, 738900, 0, 1},
  {"./lpm_lib/64/LPM_COMPONENTS.sim", "", 1058852, 173914, 0, 1},
  {"./work/64/FPMULT.sim", "", 1232766, 5600, 0, 1},
  {"./work/64/FPADDSUB.sim", "", 1238366, 6154, 0, 1},
  {"./work/64/INTTOFLOAT.sim", "", 1244520, 5380, 0, 1},
  {"./work/64/FLOATTOINT.sim", "", 1249900, 5092, 0, 1},
  {"./work/64/FLOATTOINT__NORMAL.sim", "", 1254992, 107281, 0, 1},
  {"./work/64/FPSQRT_SAFE_PATH.sim", "", 1362273, 3124, 0, 1},
  {"./work/64/FPSQRT.sim", "", 1365397, 5837, 0, 1},

  };
  static lPkgFileInfoStruct lPkgFileInfoArr[] = {
  {"linux64/packages/IEEE/lib/64/STD_LOGIC_SIGNED__.sim"},

  };
int gGetFileInfo(char *xFileName, long xTimeStamp, long *xFileOffsetPtr, size_t *xFileSizePtr, int xCheckInPkgSimFiles,  char **xRealFileName)
{
  int j, lNumOfPkgSimFiles;
  static void *ht = 0;
  static int i = 0;
  static int k = 0;
at_least_one_object_file = 1;
  lNumOfScsFiles = 23;
  lNumOfPkgSimFiles = 1;
  if (xCheckInPkgSimFiles)
  {
     for (j = 0; j < lNumOfPkgSimFiles; j++)
     {
       char* lFName;
       lFName = lPkgFileInfoArr[k].dFileName;
       if (strcmp(lFName, xFileName) == 0)
           return 0;
       k = (k + 1) % lNumOfPkgSimFiles;
     }
     return 1;
  }
  if (!ht)
  {
    ht  = kernel_scs_file_ht_new(lFInfoArr, lNumOfScsFiles);
  }
  if (ht && (kernel_scs_file_ht_get(ht, xFileName, &i) == 0))
  { /* found it! The indicator 'i' was set properly. */
    if (xRealFileName)
        *xRealFileName = lFInfoArr[i].dRealFileName;
    *xFileSizePtr = lFInfoArr[i].dFileSize;
    *xFileOffsetPtr = lFInfoArr[i].dFileOffset;
    return 0;
  }
  return 1;
}
int getNextSimFile(char **fn, long *offset)
{
  static int cur;
  for ( ; cur < lNumOfScsFiles; ) {
    if (!lFInfoArr[cur].simFlag) {
      cur++;
      continue;
    }
    *fn = lFInfoArr[cur].dFileName;
    *offset = lFInfoArr[cur].dFileOffset;
    cur++;
    return 1;
  }
  return 0;
}

#ifdef __cplusplus
}
#endif
