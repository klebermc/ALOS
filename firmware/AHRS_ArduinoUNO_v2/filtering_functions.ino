// average the values in the array:
float  mean(float arrayWithValues[]){
  float average = 0;
  int i;
  for (i = 0; i< numReadings; i++) {
    average += arrayWithValues[i]/numReadings;
  }
  return average;
}

//find the median value
float median(float arrayWithValues[]){
  // sort the array using a bubble sort:
  bubbleSort(arrayWithValues);
  return (arrayWithValues[(int)floor((float)(numReadings-1)/2)]+arrayWithValues[(int)ceil((float)(numReadings-1)/2)])/2;;
}

//sort the array with crescent values
void bubbleSort(float arrayWithValues[]) {
  int out, in;
  float swapper;
  for(out=0 ; out < numReadings; out++) {  // outer loop
    for(in=out; in<(numReadings-1); in++)  {  // inner loop
      if( arrayWithValues[in] > arrayWithValues[in+1] ) {   // out of order?
        // swap them:
        swapper = arrayWithValues[in];
        arrayWithValues [in] = arrayWithValues[in+1];
        arrayWithValues[in+1] = swapper;
      }
    }
  }
}
