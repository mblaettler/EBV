/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */
 
/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>

#define IMG_SIZE NUM_COLORS*OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT
#define NUM_COL		2

const int nc = OSC_CAM_MAX_IMAGE_WIDTH;
const int nr = OSC_CAM_MAX_IMAGE_HEIGHT;

bool ManualThreshold;

/* skip pixel at border */
const int Border = 2;

/* minimum size of objects (sum of all pixels) */
const int MinArea = 500;

/* size of centroid marker */
const int SizeCross = 10;

struct OSC_VIS_REGIONS ImgRegions;/* these contain the foreground objects */

unsigned char OtsuThreshold(int InIndex);
void Binarize(unsigned char threshold);
void Erode_3x3(int InIndex, int OutIndex);
void Dilate_3x3(int InIndex, int OutIndex);
void DetectRegions();
void DrawBoundingBoxes();
void ToYCbCr();

void ChangeDetection(int inIdx, int outIdx, int markerIdx) {
	uint8 FrgCol[NUM_COL][3] = {{52, 113, 177}, {41, 142, 116}};
	int r, c, frg, p;

	//loop over the rows
	for(r = 0; r < nr*nc; r += nc)
	{
		//loop over the columns
		for(c = 0; c < nc; c++)
		{
			//loop over the different Frg colors and find smallest difference
			int MinDif = 1 << 30;
			int MinInd = 0;

			for(frg = 0; frg < NUM_COL; frg++)
			{
				int Dif = 0;
				//loop over the color planes (r, g, b) and sum up the difference
				for(p = 1; p < NUM_COLORS; p++)
				{
					Dif += abs((int) data.u8TempImage[inIdx][(r+c)*NUM_COLORS+p]-
					(int) FrgCol[frg][p]);
				}
				if(Dif < MinDif)
				{
					MinDif = Dif;
					MinInd = frg;
				}
			}
			//if the difference is smaller than threshold value
			if(MinDif < data.ipc.state.nThreshold)
			{
				//set pixel value to 255 in THRESHOLD image for further processing
				//(we use only the first third of the image buffer)
				data.u8TempImage[markerIdx][(r+c)] = 255;
				//set pixel value to Frg color in BACKGROUND image for visualization
				for(p = 0; p < NUM_COLORS; p++)
				{
					data.u8TempImage[outIdx][(r+c)*NUM_COLORS+p] = FrgCol[MinInd][p];
				}
			}
		}
	}
}

void ResetProcess()
{
	//called when "reset" button is pressed
	if(ManualThreshold == false)
		ManualThreshold = true;
	else
		ManualThreshold = false;
}


void ProcessFrame() {
	//initialize counters
	if(data.ipc.state.nStepCounter == 1) {
		ManualThreshold = false;
	} else {

		ToYCbCr(SENSORIMG, INDEX0);

		memset(data.u8TempImage[THRESHOLD], 0, IMG_SIZE);
		memset(data.u8TempImage[BACKGROUND], 0, IMG_SIZE);
		ChangeDetection(INDEX0, BACKGROUND, THRESHOLD);

		Erode_3x3(THRESHOLD, INDEX1);
		Dilate_3x3(INDEX1, THRESHOLD);

		DetectRegions(THRESHOLD, INDEX1);

		DrawBoundingBoxes(INDEX1);
	}
}

void ToYCbCr(int inIdx, int outIdx) {
	int r, c;
	for(r = 0; r < nr*nc; r += nc) {
		//loop over the columns
		for(c = 0; c < nc; c++) {
			//get rgb values (order is actually bgr!)
			float B_ = data.u8TempImage[inIdx][(r+c)*NUM_COLORS+0];
			float G_ = data.u8TempImage[inIdx][(r+c)*NUM_COLORS+1];
			float R_ = data.u8TempImage[inIdx][(r+c)*NUM_COLORS+2];

			uint8 Y_ = (uint8) ( 0 + 0.299*R_ + 0.587*G_ + 0.114*B_);
			uint8 Cb_ = (uint8) (128 - 0.169*R_ - 0.331*G_ + 0.500*B_);
			uint8 Cr_ = (uint8) (128 + 0.500*R_ - 0.419*G_ - 0.081*B_);

			// write result to outIdx
			data.u8TempImage[outIdx][(r+c)*NUM_COLORS+0] = Y_;
			data.u8TempImage[outIdx][(r+c)*NUM_COLORS+1] = Cb_;
			data.u8TempImage[outIdx][(r+c)*NUM_COLORS+2] = Cr_;
		}
	}
}

void Erode_3x3(int InIndex, int OutIndex)
{
	int c, r;

	for(r = Border*nc; r < (nr-Border)*nc; r += nc) {
		for(c = Border; c < (nc-Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r+c];
			data.u8TempImage[OutIndex][r+c] = *(p-nc-1) & *(p-nc) & *(p-nc+1) &
											   *(p-1)    & *p      & *(p+1)    &
											   *(p+nc-1) & *(p+nc) & *(p+nc+1);
		}
	}
}

void Dilate_3x3(int InIndex, int OutIndex)
{
	int c, r;

	for(r = Border*nc; r < (nr-Border)*nc; r += nc) {
		for(c = Border; c < (nc-Border); c++) {
			unsigned char* p = &data.u8TempImage[InIndex][r+c];
			data.u8TempImage[OutIndex][r+c] = *(p-nc-1) | *(p-nc) | *(p-nc+1) |
											        *(p-1)    | *p      | *(p+1)    |
											        *(p+nc-1) | *(p+nc) | *(p+nc+1);
		}
	}
}


void DetectRegions(int inIdx, int outIdx) {
	struct OSC_PICTURE Pic;
	int i;

	//set pixel value to 1 in INDEX0 because the image MUST be binary (i.e. values of 0 and 1)
	for(i = 0; i < IMG_SIZE; i++) {
		data.u8TempImage[outIdx][i] = data.u8TempImage[inIdx][i] ? 1 : 0;
	}

	//wrap image INDEX0 in picture struct
	Pic.data = data.u8TempImage[outIdx];
	Pic.width = nc;
	Pic.height = nr;
	Pic.type = OSC_PICTURE_BINARY;

	//now do region labeling and feature extraction
	OscVisLabelBinary( &Pic, &ImgRegions);
	OscVisGetRegionProperties( &ImgRegions);
}


void DrawBoundingBoxes(int bwImage) {
	uint16 o, c, r;
	for(o = 0; o < ImgRegions.noOfObjects; o++)
	{
		if(ImgRegions.objects[o].area > MinArea)
		{
			uint32 red = 0;
			uint32 blue = 0;
			enum ObjColor col;
			struct OSC_VIS_REGIONS_RUN* currentRun = ImgRegions.objects[o].root;

			do
			{
				for(c =currentRun->startColumn; c <= currentRun->endColumn; c++)
				{
					r = currentRun->row;

					if(data.u8TempImage[bwImage][r*nc+c])
					{
						red += data.u8TempImage[SENSORIMG][(r*nc+c)*NUM_COLORS+2];
						blue += data.u8TempImage[SENSORIMG][(r*nc+c)*NUM_COLORS];
					}
				}
				currentRun = currentRun->next;
			}
			while(currentRun != NULL);

			if(red > blue)
			{
				col = RED;
			}
			else
			{
				col = BLUE;
			}

			DrawBoundingBox(ImgRegions.objects[o].bboxLeft, ImgRegions.objects[o].bboxTop,
							ImgRegions.objects[o].bboxRight, ImgRegions.objects[o].bboxBottom, false, col);

			DrawLine(ImgRegions.objects[o].centroidX-SizeCross, ImgRegions.objects[o].centroidY,
					 ImgRegions.objects[o].centroidX+SizeCross, ImgRegions.objects[o].centroidY, col);
			DrawLine(ImgRegions.objects[o].centroidX, ImgRegions.objects[o].centroidY-SizeCross,
					 ImgRegions.objects[o].centroidX, ImgRegions.objects[o].centroidY+SizeCross, col);

		}
	}
}


