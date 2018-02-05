// Please read the ready-to-localize tuturial together with this example.
// https://www.pozyx.io/Documentation/Tutorials/ready_to_localize
// v1.0 adapted from stock tutorial to print output into serial monitor that is
// readable by our c code, where lines follow format: *x,y,theta$
// Note that theta value is currently substituted with the z value
// must later replace that information
// Also changed the printed comments to reflect our actual output.
// Uploaded to Arduino currently on Everest 6/29 - BR
// v1.1 Adapted orientaiton 3d code to print theta output as
// the magnetic field strength along the z-axis
// Uploaded to Arduino on Everest 7/10 -BR

#include <Pozyx.h>
#include <Pozyx_definitions.h>
#include <Wire.h>

////////////////////////////////////////////////
////////////////// PARAMETERS //////////////////
////////////////////////////////////////////////

uint8_t num_anchors = 4;                                    // the number of anchors
uint16_t anchors[4] = {0x607B, 0x606D, 0x6055, 0x6045};     // the network id of the anchors: change these to the network ids of your anchors.
int32_t heights[4] = {2790, 2770,2790, 1875};              // anchor z-coordinates in mm
boolean bProcessing = false;                               // set this to true to output data for the processing sketch     
int16_t magn_raw[3];    

// only required for manual anchor calibration. Please change this to the coordinates measured for the anchors
int32_t anchors_x[4] = {-880, 2735, -850, 2805};              // anchor x-coorindates in mm
int32_t anchors_y[4] = {-2230, -2255, 2420, 2420};            // anchor y-coordinates in mm

////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);
  
  if(Pozyx.begin() == POZYX_FAILURE){
    Serial.println(F("ERROR: Unable to connect to POZYX shield"));
    Serial.println(F("Reset required"));
    delay(100);
    abort();
  }
  
  // clear all previous devices in the device list
  Pozyx.clearDevices();

  //manually set the anchor coordinates.
  Serial.println();
  Serial.println(F("Loading Pre-defined Anchor Coordinates: x,y,z"));
  SetAnchorsManual();

  printCalibrationResult();
  delay(3000);

  Serial.println(F("Starting positioning: "));
}

void loop()
{
  Pozyx.regRead(POZYX_MAGN_X, (uint8_t*)&magn_raw, 3*sizeof(int16_t));
  
  coordinates_t position;  
  int status = Pozyx.doPositioning(&position, POZYX_2_5D, 0);
  
  if (status == POZYX_SUCCESS)
  {
    // print out the result
    if(!bProcessing)
    {
      printCoordinates(position);
    }
    else
    {    
      printCoordinatesProcessing(position);
    }
  }
}


// function to print the coordinates to the serial monitor
void printCoordinates(coordinates_t coor)
{  
  Serial.print("*");
  Serial.print(coor.x);
  Serial.print(",");
  Serial.print(coor.y);
  Serial.print(",");
  Serial.print(magn_raw[2]);
  Serial.print("$");
  Serial.println(); 
}

// function to print out positoining data + ranges for the processing sketch
// only used if bProcessing set to true
void printCoordinatesProcessing(coordinates_t coor)
{  
  // get the network id and print it
  uint16_t network_id;
  Pozyx.getNetworkId(&network_id);
  
  Serial.print("POS,0x");
  Serial.print(network_id,HEX);
  Serial.print(",");
  Serial.print(coor.x);
  Serial.print(",");
  Serial.print(coor.y);
  Serial.print(",");
  Serial.print(coor.z);
  //Serial.print(",");
  
  // get information about the positioning error and print it
  pos_error_t pos_error;
  Pozyx.getPositionError(&pos_error);
  //below comments useful for future Processing sketch showing grid,
  // error graphs, and range data
    
  //Serial.print(pos_error.x);
  //Serial.print(",");
  //Serial.print(pos_error.y);
  //Serial.print(",");
  //Serial.print(pos_error.z);
  //Serial.print(",");
  //Serial.print(pos_error.xy);
  //Serial.print(",");
  //Serial.print(pos_error.xz);
  //Serial.print(",");
  //Serial.print(pos_error.yz); 
  
  // read out the ranges to each anchor and print it 
//  for (int i=0; i < num_anchors; i++){
//    device_range_t range;
//    Pozyx.getDeviceRangeInfo(anchors[i], &range);
//    Serial.print(",");
//    Serial.print(range.distance);  
//    Serial.print(",");
//    Serial.print(range.RSS); 
//  }
  Serial.println();
}

// print out the anchor coordinates (also required for the processing sketch)
void printCalibrationResult()
{
  uint8_t list_size;
  int status;

  status = Pozyx.getDeviceListSize(&list_size);
  
  if(list_size == 0)
  {
    Serial.println("Calibration failed.");
    Serial.println(Pozyx.getSystemError());
    return;
  }
  
  uint16_t device_ids[list_size];
  status &= Pozyx.getDeviceIds(device_ids,list_size);
  
  coordinates_t anchor_coor;
  for(int i=0; i<list_size; i++)
  {
    Serial.print("ANCHOR,");
    Serial.print("0x");
    Serial.print(device_ids[i], HEX);
    Serial.print(",");    
    status = Pozyx.getDeviceCoordinates(device_ids[i], &anchor_coor);
    Serial.print(anchor_coor.x);
    Serial.print(",");
    Serial.print(anchor_coor.y);
    Serial.print(",");
    Serial.println(anchor_coor.z);
  }    
}

// function to manually set the anchor coordinates
void SetAnchorsManual()
{
 int i=0;
 for(i=0; i<num_anchors; i++){
   device_coordinates_t anchor;
   anchor.network_id = anchors[i];
   anchor.flag = 0x1; 
   anchor.pos.x = anchors_x[i];
   anchor.pos.y = anchors_y[i];
   anchor.pos.z = heights[i];
   Pozyx.addDevice(anchor);
 }
 
}
