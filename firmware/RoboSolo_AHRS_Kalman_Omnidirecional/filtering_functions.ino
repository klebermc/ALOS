// average the values in the array:
float  mean(float arrayWithValues[], int num_of_readings){
  float average = 0;
  int i;
  for (i = 0; i< num_of_readings; i++) {
    average += arrayWithValues[i]/num_of_readings;
  }
  return average;
}

//find the median value
float median(float arrayWithValues[], int num_of_readings){
  // sort the array using a bubble sort:
  bubbleSort(arrayWithValues, num_of_readings);
  return (arrayWithValues[(int)floor((float)(num_of_readings-1)/2)]+arrayWithValues[(int)ceil((float)(num_of_readings-1)/2)])/2;;
}

//sort the array with crescent values
void bubbleSort(float arrayWithValues[], int num_of_readings) {
  int out, in;
  float swapper;
  for(out=0 ; out < num_of_readings; out++) {  // outer loop
    for(in=out; in<(num_of_readings-1); in++)  {  // inner loop
      if( arrayWithValues[in] > arrayWithValues[in+1] ) {   // out of order?
        // swap them:
        swapper = arrayWithValues[in];
        arrayWithValues [in] = arrayWithValues[in+1];
        arrayWithValues[in+1] = swapper;
      }
    }
  }
}
