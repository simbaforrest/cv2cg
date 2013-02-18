/*#============================================================================
//#
//#       Filename:  ESMlibry.h
//#
//#    Description:  Tracking structures and functions prototypes 
//#
//#            $Id: ESMlibry.h,v 1.2 2005/05/02 13:00:56 sbenhima Exp $
//#
//#         Author:  Selim BENHIMANE & Ezio MALIS
//#        Company:  INRIA, Sophia-Antipolis, FRANCE
//#          Email:  Selim.benhimane@inria.fr
//#          Email:  Ezio.Malis@inria.fr
//#
//#        Project:  ICARE
//#
//#==========================================================================*/

#ifndef __ESMLIBRY_H
#define __ESMLIBRY_H

// #####   HEADER FILE INCLUDES   #############################################

// #####   MACROS  -  LOCAL TO THIS SOURCE FILE   #############################

// #####   EXPORTED MACROS   ##################################################

// #####   EXPORTED DATA TYPES   ##############################################

// #####   EXPORTED TYPE DEFINITIONS   ########################################

// Image Structure

typedef struct imageStruct {
  int    cols;  //!< width  / cols 
  int    rows;  //!< height / rows
  int    clrs;  //!< colors
  float *data;  //!< data 
} imageStruct;

// Tracking Structure

typedef struct trackStruct {
  imageStruct** images; //!< various images  
  float**    trackdata; //!< various data
  int            miter; //!< iteration number 
  int            mprec; //!< tracking precision 
  float       homog[9]; //!< the homography computed 
} trackStruct;

// #####   EXPORTED VARIABLES   ###############################################

// #####   EXPORTED FUNCTION DECLARATIONS   ##################################

// Image Functions

int    MallImage    (imageStruct *image, int xdim, int ydim, int zdim);
int    FreeImage    (imageStruct *image);
float* GetImageData (imageStruct *image);
int    GetImageCols (imageStruct *image);
int    GetImageRows (imageStruct *image);

// Input / Output Image Functions

int SavePgm (char *filename, imageStruct *image);
int ReadPgm (char *filename, imageStruct *image);

// Tracking Functions

int MallTrack (trackStruct *trackArgs, imageStruct *image, int posx, int posy,
	       int sizex, int sizy, int miter, int mprec);
int FreeTrack (trackStruct *trackArgs);
int MakeTrack (trackStruct *trackArgs, imageStruct *image);

imageStruct* GetPatm (trackStruct *trackArgs); //!< the mask pattern
imageStruct* GetPatr (trackStruct *trackArgs); //!< the reference pattern
imageStruct* GetPatc (trackStruct *trackArgs); //!< the current pattern


#endif


