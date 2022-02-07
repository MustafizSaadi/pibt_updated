/*
 * testapp.cpp
 *
 * Purpose:
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */

#include "dummy.h"
#include<stdio.h>
//#include<ctime>
#include <time.h>
#include<iostream>
#ifndef OF

#include "app.h"
time_t t1,t2;



     
clock_t start, end;
double cpu_time_used;
     
     
//       /* Do the work. */
     
     

// Stores time in current_time


int main(int argc, char *argv[])
{
  
  // start = clock();
  std::cout<<"time_taken\n";
  
  run(argc, argv);
  
  // end = clock();
  // cpu_time_used = ((double) (end - start));
  //std::cout<<"time_taken "<<cpu_time_used/1000<<"\n";
  return 0;
}

#endif
