/*
 * Code made for handling MPU6050 measurements and send them to mobile app.
 */
#include <Wire.h>
#include <WiFi.h>

//SSID and WiFi password
const char* ssid = "dawidek";
const char* password = "123456789";

//MPU adress
int MPU = 0x68;

WiFiServer server(80);

//Offset errors, calculated specificly for one MPU module
float x_offset = 0.19, y_offset = 0, z_offset = -0.5;
float gx_offset = -0.08, gy_offset = -0.31, gz_offset = 0.18;



//function responsible for getting raw acc data from MPU 6050
void acc_raw(float *x, float *y, float *z){
  
  int16_t xo,yo,zo;
  
  // Starts with 0x32 register (ACCEL_XOUT_H) Begin transmision with acc,
  //Then requestst data from register
  Wire.beginTransmission(MPU);
  Wire.write(59); 
  Wire.endTransmission(false);
  
  //request next 6 bytes
  Wire.requestFrom(MPU, 6, true); 
  //X
  xo = ( Wire.read() << 8| Wire.read()); 
  *x = (float)xo; 
  //Y 
  yo = ( Wire.read() << 8| Wire.read());
  *y = (float)yo;
  //Z
  zo = ( Wire.read() << 8| Wire.read()); 
  *z = (float)zo;
  
  //because of 16-bit ac converter in MPU module, there is need to divide raw value by 2048
  //then to get value in [m/s^2] we need to multiply value by 9.81 and substract offset errors
  *x = *x/2048*9.81 - x_offset;
  *y = *y/2048*9.81 - y_offset;
  *z = *z/2048*9.81 - z_offset;
    
}


/*
 * Function responsible for calculating acceleration vector length
 */
float acc_vektor_length(float *x, float *y, float *z){
    
    float g;

    acc_raw(x,y,z);
    
    g = sqrt(*x**x + *y**y + *z**z);
    return g;
  
}

//responsible for getting information about start movement based on acceleration vector lenght
//threshold should be greater than 10, cause of gravity
bool activity_detection(float threshold, float *x, float *y, float *z){
  
  
  if(acc_vektor_length(x,y,z)<=threshold)
  return 1;
  else
  return 0;
  
}

/*
 * working similary to acc_raw, but there is other register number for getting data from gyroscope
 */
void raw_gyro(float *x, float *y, float *z){

    int16_t xo,yo,zo;

    
    Wire.beginTransmission(MPU);
    Wire.write(67);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    
    xo = (Wire.read() << 8 | Wire.read());
    yo = (Wire.read() << 8 | Wire.read()); 
    zo = (Wire.read() << 8 | Wire.read());
    


   *x = (float)xo/16.384 + gx_offset;
   *y = (float)yo/16.384 + gy_offset;
   *z = (float)zo/16.384 + gz_offset;
  
}








void setup() {
  
  //Begin serial communication
  Serial.begin(115200);

  //Begin I2C communication
  Wire.begin();

  //Set higher I2C speed
  Wire.setClock(400000);

  /* 
   *  Acc and gyro configuration
   */
  Wire.beginTransmission(MPU);//set meas mode
  Wire.write(0x6B); 
  Wire.write(0x00); 
  Wire.endTransmission(true);
  delay(20);

  Wire.beginTransmission(MPU);//set the highest dps range 
  Wire.write(27);
  Wire.write(0b00011010);
  Wire.endTransmission(true);
  delay(10);

  Wire.beginTransmission(MPU);//set the highest range
  Wire.write(28);
  Wire.write(0b00011010);
  Wire.endTransmission(true);
  delay(10);

  Wire.beginTransmission(MPU);//set the highest sample rate
  Wire.write(29);
  Wire.write(0b00001000);
  Wire.endTransmission(true);
  delay(10);
  /*
   * end of configuration
   */
   
  //wi-fi connection
  Serial.println("Connecting...");
  Serial.print(ssid);
  WiFi.begin(ssid, password);

  //waiting for connection
  while (WiFi.status() != WL_CONNECTED){
    
  delay(500);
  Serial.print(".");
  
  }
  
  //showing ip adress
  Serial.println(WiFi.localIP());
  
  //activate server
  server.begin();

}

void loop() {

  //package for raw data
  String paczka = "";

  //iterator neccesary for increasing size of tables
  int i = 0;

  //is checking if server is available
  WiFiClient client = server.available();
  if(!client){
    
    return;//return to start of main loop
    
  }
  
  //waiting for signal from mobile app
  while(!client.available()){
    
    delay(1);
    
  }

  //reading a message from app
  String request = client.readStringUntil('\r');

   if(request.indexOf("cios")!= -1){

      //pointers on first elements of tables holding acc and gyro raw data
      float* accx=(float*)malloc(sizeof(float)),*accy=(float*)malloc(sizeof(float)),*accz=(float*)malloc(sizeof(float));
      float* gyrox=(float*)malloc(sizeof(float)),*gyroy=(float*)malloc(sizeof(float)),*gyroz=(float*)malloc(sizeof(float));

      
      float roll, pitch, yaw;

      
      //calculating a base orientation of fist
      acc_raw(&accx[0], &accy[0], &accz[0]);
      roll = atan2(accy[0], accz[0]);
      pitch = -atan2(accx[0], sqrt(accz[0] * accz[0] + accy[0] * accy[0])); 
      yaw = 0; //base yaw is matched with direction of x axis

      
      paczka = "r";
      paczka.concat(roll);
      paczka.concat("p");
      paczka.concat(pitch);
      paczka.concat("y");
      paczka.concat(yaw);


      //go to sampling acc data after acceleration i grater than treshold
      int timer = 0;
      while(activity_detection(11,&accx[0], &accy[0], &accz[0])){

        if(millis() - timer>0.5){

          raw_gyro(&gyrox[0], &gyroy[0], &gyroz[0]);
          timer = millis();
          
        }
      }

    
    i++;
    accx = (float*)realloc(accx,(i+1)*sizeof(float));
    accy = (float*)realloc(accy,(i+1)*sizeof(float));
    accz = (float*)realloc(accz,(i+1)*sizeof(float));
    gyrox = (float*)realloc(gyrox,(i+1)*sizeof(float));
    gyroy = (float*)realloc(gyroy,(i+1)*sizeof(float));
    gyroz = (float*)realloc(gyroz,(i+1)*sizeof(float));


    //sampling state
    timer = 0;
    int timer2 = millis();
    while(millis() - timer2 <= 1000){//sampling for one second

      if(millis() - timer > 0.5){
        
        acc_raw(&accx[i],&accy[i],&accz[i]);
        raw_gyro(&gyrox[i],&gyroy[i],&gyroz[i]);

        i++;
        accx = (float*)realloc(accx,(i+1)*sizeof(float));
        accy = (float*)realloc(accy,(i+1)*sizeof(float));
        accz = (float*)realloc(accz,(i+1)*sizeof(float));
        gyrox = (float*)realloc(gyrox,(i+1)*sizeof(float));
        gyroy = (float*)realloc(gyroy,(i+1)*sizeof(float));
        gyroz = (float*)realloc(gyroz,(i+1)*sizeof(float));
        timer = millis();
       
      }
    }

    for(int j = 0; j<i; j++){

      paczka.concat("ax");
      paczka.concat(accx[j]);
      paczka.concat("ay");
      paczka.concat(accy[j]);
      paczka.concat("az");
      paczka.concat(accz[j]);
      paczka.concat("gx");
      paczka.concat(gyrox[j]);
      paczka.concat("gy");
      paczka.concat(gyroy[j]);
      paczka.concat("gz");
      paczka.concat(gyroz[j]);
      
    }

    paczka.concat("END");
    paczka.concat(i);


    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println("");
    client.print(paczka);

      //czyszczenie dynamicznie zaalokowanych zasobów
    free(accx);free(accy);free(accz);
    free(gyrox);free(gyroy);free(gyroz);

    paczka = "";
    
   }

}
