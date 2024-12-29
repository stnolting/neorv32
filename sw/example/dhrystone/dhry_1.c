/*
 ****************************************************************************
 *
 *                   "DHRYSTONE" Benchmark Program
 *                   -----------------------------
 *
 *  Version:    C, Version 2.1
 *
 *  File:       dhry_1.c (part 2 of 3)
 *
 *  Date:       May 25, 1988
 *
 *  Author:     Reinhold P. Weicker
 *
 ****************************************************************************
 */

/*
  Original from https://github.com/sifive/benchmark-dhrystone
  Modified for the NEORV32 RISC-V Processor
*/

#define TIME

#include <neorv32.h>
#include <string.h>
#include "dhry.h"

/** UART BAUD rate */
#define BAUD_RATE   19200

#ifndef DHRY_ITERS
#define DHRY_ITERS 10000
#endif

/* Global Variables: */

Rec_Pointer     Ptr_Glob,
                Next_Ptr_Glob;
int             Int_Glob;
Boolean         Bool_Glob;
char            Ch_1_Glob,
                Ch_2_Glob;
int             Arr_1_Glob [50];
int             Arr_2_Glob [50] [50];

//extern char     *malloc ();
Enumeration     Func_1 ();
  /* forward declaration necessary since Enumeration may not simply be int */

#ifndef REG
        Boolean Reg = false;
#define REG
        /* REG becomes defined as empty */
        /* i.e. no register variables   */
#else
        Boolean Reg = true;
#endif

/* variables for time measurement: */

#ifdef TIMES
struct tms      time_info;
extern  int     times ();
                /* see library function "times" */
#define Too_Small_Time (2*HZ)
                /* Measurements should last at least about 2 seconds */
#endif
#ifdef TIME
extern long     time();
                /* see library function "time"  */
#define Too_Small_Time 2
                /* Measurements should last at least 2 seconds */
#endif
#ifdef MSC_CLOCK
extern clock_t	clock();
#define Too_Small_Time (2*HZ)
#endif

long            Begin_Time,
                End_Time,
                User_Time;
float           Microseconds,
                Dhrystones_Per_Second;

/* end of variables for time measurement */


int main (void)
/*****/

  /* main program, corresponds to procedures        */
  /* Main and Proc_0 in the Ada version             */
{
        One_Fifty       Int_1_Loc;
  REG   One_Fifty       Int_2_Loc;
        One_Fifty       Int_3_Loc;
  REG   char            Ch_Index;
        Enumeration     Enum_Loc;
        Str_30          Str_1_Loc;
        Str_30          Str_2_Loc;
  REG   int             Run_Index;
  REG   int             Number_Of_Runs;


  { /* *****  NEORV32-SPECIFIC ***** */
    neorv32_rte_setup();
    neorv32_cpu_csr_write(CSR_MIE, 0); // no interrupts
    neorv32_uart0_setup(BAUD_RATE, 0);

    neorv32_uart0_printf("NEORV32: Processor running at %u Hz\n", (uint32_t)neorv32_sysinfo_get_clk());
    neorv32_uart0_printf("NEORV32: Executing Dhrystone (%u iterations). This may take some time...\n\n", (uint32_t)DHRY_ITERS);

    // clear cycle counter
    neorv32_cpu_set_mcycle(0);

#ifndef RUN_DHRYSTONE
    #warning DHRYSTONE HAS NOT BEEN COMPILED! Use >>make USER_FLAGS+=-DRUN_DHRYSTONE clean_all exe<< to compile it.

    // inform the user if you are actually executing this
    neorv32_uart0_printf("ERROR! DhryStone has not been compiled. Use >>make USER_FLAGS+=-DRUN_DHRYSTONE ...<< to compile it.\n");

    while(1);
#endif
  } /* ***** /NEORV32-SPECIFIC ***** */


  /* Initializations */
  { /* *****  NEORV32-SPECIFIC ***** */
    // use static memory allocation, no dynamic malloc
    //Next_Ptr_Glob = (Rec_Pointer) malloc (sizeof (Rec_Type));
    //Ptr_Glob = (Rec_Pointer) malloc (sizeof (Rec_Type));
    static Rec_Type Rec_Type_v0, Rec_Type_v1;
    Next_Ptr_Glob = &Rec_Type_v0;
    Ptr_Glob = &Rec_Type_v1;
  } /* ***** /NEORV32-SPECIFIC ***** */


  Ptr_Glob->Ptr_Comp                    = Next_Ptr_Glob;
  Ptr_Glob->Discr                       = Ident_1;
  Ptr_Glob->variant.var_1.Enum_Comp     = Ident_3;
  Ptr_Glob->variant.var_1.Int_Comp      = 40;
  strcpy (Ptr_Glob->variant.var_1.Str_Comp,
          "DHRYSTONE PROGRAM, SOME STRING");
  strcpy (Str_1_Loc, "DHRYSTONE PROGRAM, 1'ST STRING");

  Arr_2_Glob [8][7] = 10;
        /* Was missing in published program. Without this statement,    */
        /* Arr_2_Glob [8][7] would have an undefined value.             */
        /* Warning: With 16-Bit processors and Number_Of_Runs > 32000,  */
        /* overflow may occur for this array element.                   */

  neorv32_uart0_printf ("\n");
  neorv32_uart0_printf ("Dhrystone Benchmark, Version 2.1 (Language: C)\n");
  neorv32_uart0_printf ("\n");
  if (Reg)
  {
    neorv32_uart0_printf ("Program compiled with 'register' attribute\n");
    neorv32_uart0_printf ("\n");
  }
  else
  {
    neorv32_uart0_printf ("Program compiled without 'register' attribute\n");
    neorv32_uart0_printf ("\n");
  }
#ifdef DHRY_ITERS
  Number_Of_Runs = DHRY_ITERS;
#else
  neorv32_uart0_printf ("Please give the number of runs through the benchmark: ");
  {
    int n;
    scanf ("%d", &n);
    Number_Of_Runs = n;
  }
  neorv32_uart0_printf ("\n");
#endif

  neorv32_uart0_printf ("Execution starts, %u runs through Dhrystone\n", (uint32_t)Number_Of_Runs);

  /***************/
  /* Start timer */
  /***************/

/*
#ifdef TIMES
  times (&time_info);
  Begin_Time = (long) time_info.tms_utime;
#endif
#ifdef TIME
  Begin_Time = time ( (long *) 0);
#endif
#ifdef MSC_CLOCK
  Begin_Time = clock();
#endif
*/

  { /* *****  NEORV32-SPECIFIC ***** */
    Begin_Time = (long)neorv32_clint_time_get();
  } /* ***** /NEORV32-SPECIFIC ***** */

  for (Run_Index = 1; Run_Index <= Number_Of_Runs; ++Run_Index)
  {

    Proc_5();
    Proc_4();
      /* Ch_1_Glob == 'A', Ch_2_Glob == 'B', Bool_Glob == true */
    Int_1_Loc = 2;
    Int_2_Loc = 3;
    strcpy (Str_2_Loc, "DHRYSTONE PROGRAM, 2'ND STRING");
    Enum_Loc = Ident_2;
    Bool_Glob = ! Func_2 (Str_1_Loc, Str_2_Loc);
      /* Bool_Glob == 1 */
    while (Int_1_Loc < Int_2_Loc)  /* loop body executed once */
    {
      Int_3_Loc = 5 * Int_1_Loc - Int_2_Loc;
        /* Int_3_Loc == 7 */
      Proc_7 (Int_1_Loc, Int_2_Loc, &Int_3_Loc);
        /* Int_3_Loc == 7 */
      Int_1_Loc += 1;
    } /* while */
      /* Int_1_Loc == 3, Int_2_Loc == 3, Int_3_Loc == 7 */
    Proc_8 (Arr_1_Glob, Arr_2_Glob, Int_1_Loc, Int_3_Loc);
      /* Int_Glob == 5 */
    Proc_1 (Ptr_Glob);
    for (Ch_Index = 'A'; Ch_Index <= Ch_2_Glob; ++Ch_Index)
                             /* loop body executed twice */
    {
      if (Enum_Loc == Func_1 (Ch_Index, 'C'))
          /* then, not executed */
        {
        Proc_6 (Ident_1, &Enum_Loc);
        strcpy (Str_2_Loc, "DHRYSTONE PROGRAM, 3'RD STRING");
        Int_2_Loc = Run_Index;
        Int_Glob = Run_Index;
        }
    }
      /* Int_1_Loc == 3, Int_2_Loc == 3, Int_3_Loc == 7 */
    Int_2_Loc = Int_2_Loc * Int_1_Loc;
    Int_1_Loc = Int_2_Loc / Int_3_Loc;
    Int_2_Loc = 7 * (Int_2_Loc - Int_3_Loc) - Int_1_Loc;
      /* Int_1_Loc == 1, Int_2_Loc == 13, Int_3_Loc == 7 */
    Proc_2 (&Int_1_Loc);
      /* Int_1_Loc == 5 */

  } /* loop "for Run_Index" */

  /**************/
  /* Stop timer */
  /**************/

/*
#ifdef TIMES
  times (&time_info);
  End_Time = (long) time_info.tms_utime;
#endif
#ifdef TIME
  End_Time = time ( (long *) 0);
#endif
#ifdef MSC_CLOCK
  End_Time = clock();
#endif
*/

  { /* *****  NEORV32-SPECIFIC ***** */
    End_Time = (long)neorv32_clint_time_get();
  } /* ***** /NEORV32-SPECIFIC ***** */


  neorv32_uart0_printf ("Execution ends\n");
  neorv32_uart0_printf ("\n");
  neorv32_uart0_printf ("Final values of the variables used in the benchmark:\n");
  neorv32_uart0_printf ("\n");
  neorv32_uart0_printf ("Int_Glob:            %u\n", (uint32_t)Int_Glob);
  neorv32_uart0_printf ("        should be:   %u\n", 5);
  neorv32_uart0_printf ("Bool_Glob:           %u\n", (uint32_t)Bool_Glob);
  neorv32_uart0_printf ("        should be:   %u\n", 1);
  neorv32_uart0_printf ("Ch_1_Glob:           %c\n", (uint32_t)Ch_1_Glob);
  neorv32_uart0_printf ("        should be:   %c\n", 'A');
  neorv32_uart0_printf ("Ch_2_Glob:           %c\n", (uint32_t)Ch_2_Glob);
  neorv32_uart0_printf ("        should be:   %c\n", 'B');
  neorv32_uart0_printf ("Arr_1_Glob[8]:       %u\n", (uint32_t)Arr_1_Glob[8]);
  neorv32_uart0_printf ("        should be:   %u\n", 7);
  neorv32_uart0_printf ("Arr_2_Glob[8][7]:    %u\n", (uint32_t)Arr_2_Glob[8][7]);
  neorv32_uart0_printf ("        should be:   Number_Of_Runs + 10\n");
  neorv32_uart0_printf ("Ptr_Glob->\n");
  neorv32_uart0_printf ("  Ptr_Comp:          %u\n", (uint32_t) Ptr_Glob->Ptr_Comp);
  neorv32_uart0_printf ("        should be:   (implementation-dependent)\n");
  neorv32_uart0_printf ("  Discr:             %u\n", (uint32_t)Ptr_Glob->Discr);
  neorv32_uart0_printf ("        should be:   %u\n", 0);
  neorv32_uart0_printf ("  Enum_Comp:         %u\n", (uint32_t)Ptr_Glob->variant.var_1.Enum_Comp);
  neorv32_uart0_printf ("        should be:   %u\n", 2);
  neorv32_uart0_printf ("  Int_Comp:          %u\n", (uint32_t)Ptr_Glob->variant.var_1.Int_Comp);
  neorv32_uart0_printf ("        should be:   %u\n", 17);
  neorv32_uart0_printf ("  Str_Comp:          %s\n", Ptr_Glob->variant.var_1.Str_Comp);
  neorv32_uart0_printf ("        should be:   DHRYSTONE PROGRAM, SOME STRING\n");
  neorv32_uart0_printf ("Next_Ptr_Glob->\n");
  neorv32_uart0_printf ("  Ptr_Comp:          %u\n", (uint32_t) Next_Ptr_Glob->Ptr_Comp);
  neorv32_uart0_printf ("        should be:   (implementation-dependent), same as above\n");
  neorv32_uart0_printf ("  Discr:             %u\n", (uint32_t)Next_Ptr_Glob->Discr);
  neorv32_uart0_printf ("        should be:   %u\n", 0);
  neorv32_uart0_printf ("  Enum_Comp:         %u\n", (uint32_t)Next_Ptr_Glob->variant.var_1.Enum_Comp);
  neorv32_uart0_printf ("        should be:   %u\n", 1);
  neorv32_uart0_printf ("  Int_Comp:          %u\n", (uint32_t)Next_Ptr_Glob->variant.var_1.Int_Comp);
  neorv32_uart0_printf ("        should be:   %u\n", 18);
  neorv32_uart0_printf ("  Str_Comp:          %s\n",
                                Next_Ptr_Glob->variant.var_1.Str_Comp);
  neorv32_uart0_printf ("        should be:   DHRYSTONE PROGRAM, SOME STRING\n");
  neorv32_uart0_printf ("Int_1_Loc:           %u\n", (uint32_t)Int_1_Loc);
  neorv32_uart0_printf ("        should be:   %u\n", 5);
  neorv32_uart0_printf ("Int_2_Loc:           %u\n", (uint32_t)Int_2_Loc);
  neorv32_uart0_printf ("        should be:   %u\n", 13);
  neorv32_uart0_printf ("Int_3_Loc:           %u\n", (uint32_t)Int_3_Loc);
  neorv32_uart0_printf ("        should be:   %u\n", 7);
  neorv32_uart0_printf ("Enum_Loc:            %u\n", (uint32_t)Enum_Loc);
  neorv32_uart0_printf ("        should be:   %u\n", 1);
  neorv32_uart0_printf ("Str_1_Loc:           %s\n", Str_1_Loc);
  neorv32_uart0_printf ("        should be:   DHRYSTONE PROGRAM, 1'ST STRING\n");
  neorv32_uart0_printf ("Str_2_Loc:           %s\n", Str_2_Loc);
  neorv32_uart0_printf ("        should be:   DHRYSTONE PROGRAM, 2'ND STRING\n");
  neorv32_uart0_printf ("\n");

  User_Time = End_Time - Begin_Time;

//  if (User_Time < Too_Small_Time)
//  {
//    neorv32_uart0_printf ("Measured time too small to obtain meaningful results\n");
//    neorv32_uart0_printf ("Please increase number of runs\n");
//    neorv32_uart0_printf ("\n");
//  }
//  else
  {
/*
#ifdef TIME
    Microseconds = (float) User_Time * Mic_secs_Per_Second
                        / (float) Number_Of_Runs;
    Dhrystones_Per_Second = (float) Number_Of_Runs / (float) User_Time;
#else
    Microseconds = (float) User_Time * Mic_secs_Per_Second
                        / ((float) HZ * ((float) Number_Of_Runs));
    Dhrystones_Per_Second = ((float) HZ * (float) Number_Of_Runs)
                        / (float) User_Time;
#endif
*/
    { /* *****  NEORV32-SPECIFIC ***** */
      neorv32_uart0_printf ("Microseconds for one run through Dhrystone: %u \n", (uint32_t)((User_Time * (Mic_secs_Per_Second / Number_Of_Runs)) / neorv32_sysinfo_get_clk()));

      uint32_t dhry_per_sec = (uint32_t)(neorv32_sysinfo_get_clk() / (User_Time / Number_Of_Runs));

      neorv32_uart0_printf ("Dhrystones per Second:                      %u \n\n", (uint32_t)dhry_per_sec);

      neorv32_uart0_printf("NEORV32: << DETAILED RESULTS (integer parts only) >>\n");
      neorv32_uart0_printf("NEORV32: Total cycles:      %u\n", (uint32_t)User_Time);
      neorv32_uart0_printf("NEORV32: Cycles per second: %u\n", (uint32_t)neorv32_sysinfo_get_clk());
      neorv32_uart0_printf("NEORV32: Total runs:        %u\n", (uint32_t)Number_Of_Runs);

      neorv32_uart0_printf("\n");
      neorv32_uart0_printf("NEORV32: DMIPS/s:           %u\n", (uint32_t)dhry_per_sec);
      neorv32_uart0_printf("NEORV32: DMIPS/s/MHz:       %u\n", (uint32_t)(dhry_per_sec / (neorv32_sysinfo_get_clk() / 1000000)));

      neorv32_uart0_printf("\n");
      neorv32_uart0_printf("NEORV32: VAX DMIPS/s:       %u\n", (uint32_t)dhry_per_sec/1757);
      neorv32_uart0_printf("NEORV32: VAX DMIPS/s/MHz:   %u/1757\n", (uint32_t)(dhry_per_sec / (neorv32_sysinfo_get_clk() / 1000000)));
    } /* ***** /NEORV32-SPECIFIC ***** */
    /*
      neorv32_uart0_printf ("Microseconds for one run through Dhrystone: ");
      //neorv32_uart0_printf ("%6.1f \n", Microseconds);
      neorv32_uart0_printf ("%d \n", (int)Microseconds);
      neorv32_uart0_printf ("Dhrystones per Second:                      ");
      //neorv32_uart0_printf ("%6.1f \n", Dhrystones_Per_Second);
      neorv32_uart0_printf ("%d \n", (int)Dhrystones_Per_Second);
      neorv32_uart0_printf ("\n");
    */
  }

  return 0;
}


void Proc_1 (Ptr_Val_Par)
/******************/

REG Rec_Pointer Ptr_Val_Par;
    /* executed once */
{
  REG Rec_Pointer Next_Record = Ptr_Val_Par->Ptr_Comp;
                                        /* == Ptr_Glob_Next */
  /* Local variable, initialized with Ptr_Val_Par->Ptr_Comp,    */
  /* corresponds to "rename" in Ada, "with" in Pascal           */

  structassign (*Ptr_Val_Par->Ptr_Comp, *Ptr_Glob);
  Ptr_Val_Par->variant.var_1.Int_Comp = 5;
  Next_Record->variant.var_1.Int_Comp
        = Ptr_Val_Par->variant.var_1.Int_Comp;
  Next_Record->Ptr_Comp = Ptr_Val_Par->Ptr_Comp;
  Proc_3 (&Next_Record->Ptr_Comp);
    /* Ptr_Val_Par->Ptr_Comp->Ptr_Comp
                        == Ptr_Glob->Ptr_Comp */
  if (Next_Record->Discr == Ident_1)
    /* then, executed */
  {
    Next_Record->variant.var_1.Int_Comp = 6;
    Proc_6 (Ptr_Val_Par->variant.var_1.Enum_Comp,
           &Next_Record->variant.var_1.Enum_Comp);
    Next_Record->Ptr_Comp = Ptr_Glob->Ptr_Comp;
    Proc_7 (Next_Record->variant.var_1.Int_Comp, 10,
           &Next_Record->variant.var_1.Int_Comp);
  }
  else /* not executed */
    structassign (*Ptr_Val_Par, *Ptr_Val_Par->Ptr_Comp);
} /* Proc_1 */


void Proc_2 (Int_Par_Ref)
/******************/
    /* executed once */
    /* *Int_Par_Ref == 1, becomes 4 */

One_Fifty   *Int_Par_Ref;
{
  One_Fifty  Int_Loc;
  Enumeration   Enum_Loc;

  Int_Loc = *Int_Par_Ref + 10;
  do /* executed once */
    if (Ch_1_Glob == 'A')
      /* then, executed */
    {
      Int_Loc -= 1;
      *Int_Par_Ref = Int_Loc - Int_Glob;
      Enum_Loc = Ident_1;
    } /* if */
  while (Enum_Loc != Ident_1); /* true */
} /* Proc_2 */


void Proc_3 (Ptr_Ref_Par)
/******************/
    /* executed once */
    /* Ptr_Ref_Par becomes Ptr_Glob */

Rec_Pointer *Ptr_Ref_Par;

{
  if (Ptr_Glob != Null)
    /* then, executed */
    *Ptr_Ref_Par = Ptr_Glob->Ptr_Comp;
  Proc_7 (10, Int_Glob, &Ptr_Glob->variant.var_1.Int_Comp);
} /* Proc_3 */


void Proc_4 (void) /* without parameters */
/*******/
    /* executed once */
{
  Boolean Bool_Loc;

  Bool_Loc = Ch_1_Glob == 'A';
  Bool_Glob = Bool_Loc | Bool_Glob;
  Ch_2_Glob = 'B';
} /* Proc_4 */


void Proc_5 (void) /* without parameters */
/*******/
    /* executed once */
{
  Ch_1_Glob = 'A';
  Bool_Glob = false;
} /* Proc_5 */


        /* Procedure for the assignment of structures,          */
        /* if the C compiler doesn't support this feature       */
#ifdef  NOSTRUCTASSIGN
memcpy (d, s, l)
register char   *d;
register char   *s;
register int    l;
{
        while (l--) *d++ = *s++;
}
#endif


/* Compare S1 and S2, returning less than, equal to or
   greater than zero if S1 is lexicographically less than,
   equal to or greater than S2. [from glibc] */
int strcmp(const char *p1, const char *p2)
{
  const unsigned char *s1 = (const unsigned char *) p1;
  const unsigned char *s2 = (const unsigned char *) p2;
  unsigned char c1, c2;
  do
    {
      c1 = (unsigned char) *s1++;
      c2 = (unsigned char) *s2++;
      if (c1 == '\0')
        return c1 - c2;
    }
  while (c1 == c2);
  return c1 - c2;
}
