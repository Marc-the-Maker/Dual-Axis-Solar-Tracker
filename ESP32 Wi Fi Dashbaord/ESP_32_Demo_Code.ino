#include <WiFi.h>
#include <PubSubClient.h>

#define LED 2

// WiFi
const char *ssid = "Marco’s Iphone SE"; // Enter your WiFi name
const char *password = "Golf2143";  // Enter WiFi password

// MQTT Broker
const char *mqtt_broker = "172.20.10.9";
const char *topic = "home";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;

const char *topic_state_change = "state_change";
const char *topic_home = "home";
const char *topic_force_track = "force_track";
const char *topic_force_move = "force_move";
const char *topic_force_move_azi = "force_azi";
const char *topic_force_move_vert = "force_vert";
const char *topic_stop = "stop";
const char *topic_azi_time = "azi_time";
const char *topic_vert_time = "vert_time";
const char *topic_scan_start = "scan_start";
const char *topic_panel_power_scan = "panel_power_scan";
const char *topic_disable_tracker = "disable_tracker";
const char *topic_current_setpoint = "current_setpoint";
const char *topic_cc_activate = "cc_activate";
const char *topic_cv_activate = "cv_activate";
const char *topic_voltage_setpoint = "voltage_setpoint";


uint8_t AZI_Hour_Track = 0;
uint8_t AZI_Minute_Track = 0;
uint8_t AZI_Second_Track = 0;

uint8_t VERT_Hour_Track = 0;
uint8_t VERT_Minute_Track = 0;
uint8_t VERT_Second_Track = 0;

uint8_t tx_buff[5];
bool waiting = 0;

uint16_t PV_int;
uint16_t PI_int;
uint16_t BV_int;
uint16_t BI_int;
uint16_t SA_int;
uint16_t PA_int;
uint16_t SVert_int;
uint16_t PVert_int;
uint16_t P_Duty;

float PV;
float PC;
float BV;
float BI;
float SA;
float PA;
float SVert;
float PVert;
char buffer[20]; 
char timeString[9]; // Adjust the size based on your needs

long lastMsg = 0;

int state = 99;
byte buff[17];

WiFiClient espClient;
PubSubClient client(espClient);

bool home_flag = 0;
bool trigger_flag = 0;
bool disturb_flag = 0;
bool stop_flag = 0;
bool scan_flag = 0;
byte disable_tracker = 0;
bool disable_tracker_flag = 0;
bool current_setpoint_flag = 0;
double current_setpoint = 0.0;
byte CV_activate = 0;
byte CC_activate = 0;
bool CV_flag = 0;
bool CC_flag = 0;
byte battery_State = 99;

bool voltage_setpoint_flag = 0;
double voltage_setpoint = 0.0;

int azi_setpoint = 0;
int vert_setpoint = 0;

double panel_power = 0.0;
double battery_power = 0.0;

void setup() {
  pinMode(LED, OUTPUT);
 // Set software serial baud to 115200;
 Serial.begin(115200);


 // connecting to a WiFi network
 WiFi.begin(ssid, password);
 while (WiFi.status() != WL_CONNECTED) {
     delay(500);
    //  Serial.println("Connecting to WiFi..");
 }

  //Serial.println("Connected to the WiFi network");
 //connecting to a mqtt broker


 client.setServer(mqtt_broker, mqtt_port);
 client.setCallback(callback);
 while (!client.connected()) {
     String client_id = "esp32-client-";
     client_id += String(WiFi.macAddress());
    //  Serial.printf("The client %s connects to the public mqtt broker\n", client_id.c_str());
     if (client.connect(client_id.c_str())) {
        //  Serial.println("Public emqx mqtt broker connected");
     } else {
        //  Serial.print("failed with state ");
        //  Serial.print(client.state());
         delay(2000);
     }
 }
 // publish and subscribe

 client.subscribe(topic);

 client.subscribe(topic);
 client.subscribe(topic_force_move_azi);
 client.subscribe(topic_force_move_vert);
 client.subscribe(topic_stop);
 client.subscribe(topic_azi_time);
 client.subscribe(topic_vert_time);

 client.subscribe(topic_scan_start);
 client.subscribe(topic_panel_power_scan);
 client.subscribe(topic_disable_tracker);
 client.subscribe(topic_current_setpoint);
 client.subscribe(topic_voltage_setpoint);

 client.subscribe(topic_cc_activate);
 client.subscribe(topic_cv_activate);
 
 
 topic = "charging_state";
 client.subscribe(topic);
 topic = "panel_voltage";
 client.subscribe(topic);
 topic = "panel_current";
 client.subscribe(topic);
 topic = "panel_power";
 client.subscribe(topic);

 topic = "battery_voltage";
 client.subscribe(topic);
 topic = "battery_current";
 client.subscribe(topic);
 topic = "battery_power";
 client.subscribe(topic);

 topic = "panel_azi";
 client.subscribe(topic);
 topic = "panel_vert";
 client.subscribe(topic);
 topic = "sun_azi";
 client.subscribe(topic);
 topic = "sun_vert";
 client.subscribe(topic);
 delay(2000);
 client.publish("state_confirm", "1");
}

void transmit(int CMD, int D0, int D1, int D2, int D3) {
  tx_buff[0] = CMD;
  tx_buff[1] = D0;
  tx_buff[2] = D1;
  tx_buff[3] = D2;
  tx_buff[4] = D3;
  Serial.write(tx_buff, 5);
  return;  
}

void callback(char *topic, byte *payload, unsigned int length) {
  if(strcmp(topic, topic_state_change) == 0){
    payload[length] = '\0';
    state = 99;
    state = atoi((char *)payload);
  }
  else if(strcmp(topic, topic_home) == 0){
    home_flag = 1;
  }
  else if(strcmp(topic, topic_force_track) == 0){
    trigger_flag = 1;
  }
  else if(strcmp(topic, topic_force_move_azi) == 0){
    payload[length] = '\0';
    azi_setpoint = atoi((char *)payload);
  }
  else if(strcmp(topic, topic_force_move_vert) == 0){
    payload[length] = '\0';
    vert_setpoint = atoi((char *)payload);
  }
  else if(strcmp(topic, topic_stop) == 0){
    stop_flag = 1;
  }
  else if(strcmp(topic, topic_force_move) == 0){
    disturb_flag = 1;
  }
  else if(strcmp(topic, topic_scan_start) == 0){
    scan_flag = 1;
  }
  else if(strcmp(topic, topic_cc_activate) == 0){
    payload[length] = '\0';
    CC_activate = atoi((char *)payload);
    CC_flag = 1;
  }
  else if(strcmp(topic, topic_cv_activate) == 0){
    payload[length] = '\0';
    CV_activate = atoi((char *)payload);
    CV_flag = 1;
  }
  else if(strcmp(topic, topic_current_setpoint) == 0){
    String message = "";
    for (int i = 0; i < length; i++) {
      message += (char)payload[i];
    }
    current_setpoint = atof(message.c_str());
    current_setpoint_flag = 1;
  }
  else if(strcmp(topic, topic_voltage_setpoint) == 0){
    String message = "";
    for (int i = 0; i < length; i++) {
      message += (char)payload[i];
    }
    voltage_setpoint = atof(message.c_str());
    voltage_setpoint_flag = 1;
  }
  else if(strcmp(topic, topic_disable_tracker) == 0){
    String message = "";
    for (int i = 0; i < length; i++) {
      message += (char)payload[i];
    }

    disable_tracker_flag = 1;
    if (message.equals("enable")) {
      disable_tracker = 0;
    } else if (message.equals("disable")) {
      disable_tracker = 1;
    }
  }
}


void loop() {
  client.loop();
  // transmit(0x10, 0x00, 0xFF, 0xFF, 0xFF);
  // delay(1000);
// Adjust the size based on your needs
  // Serial.println(state);

  while(state == 0){
    // client.loop();
    
    // digitalWrite(LED, 1);
    // delay(1000);
    // digitalWrite(LED, 0);
    transmit(0x10, 0x00, 0xFF, 0xFF, 0xFF);

    waiting = 1;
    
    while(waiting == 1){
      if(Serial.available() > 0){
        Serial.readBytes(buff, 2);
        if((buff[1] == 0x00) && (buff[0] == 0x21)){
          waiting = 0;
          digitalWrite(LED, 1);
          delay(1000);
          digitalWrite(LED, 0);
          client.publish("state_confirm", "0");
          state = 2;
        }
      }
    }
  }


  

  while(state == 2){ //General Mode transition
    client.loop();
    
    if(home_flag == 1){
      home_flag = 0; 
      digitalWrite(LED, 1);
      delay(1000);
      digitalWrite(LED, 0);      

      transmit(0x11, 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else if(trigger_flag == 1){
      trigger_flag = 0;
      transmit(0x12, 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else if(disturb_flag == 1){
      disturb_flag = 0;
      transmit(0x13, (byte)(azi_setpoint>>8), (byte)(azi_setpoint), (byte)(vert_setpoint>>8), (byte)(vert_setpoint));
    }
    else if(stop_flag == 1){
      stop_flag = 0;
      //delay(200);
      transmit(0x14, 0xFF, 0xFF, 0xFF, 0xFF);
      digitalWrite(LED, 1);
      delay(1000);
      digitalWrite(LED, 0);
    }
    else if(scan_flag == 1){
      scan_flag = 0;
      transmit(0x15, 0xFF, 0xFF, 0xFF, 0xFF);
      digitalWrite(LED, 1);
      delay(1000);
      digitalWrite(LED, 0);
    }
    else if(disable_tracker_flag == 1){
      disable_tracker_flag = 0;
      transmit(0x16, disable_tracker, 0xFF, 0xFF, 0xFF);
    }
    else if(current_setpoint_flag == 1){
      current_setpoint_flag = 0;
      
      byte current_tx = (byte)round(current_setpoint*10);
      transmit(0x17, current_tx, 0xFF, 0xFF, 0xFF);
    }
    else if(voltage_setpoint_flag == 1){
      voltage_setpoint_flag = 0;
      
      byte current_tx = (byte)round(voltage_setpoint*10);
      transmit(0x18, current_tx, 0xFF, 0xFF, 0xFF);
    }
    else if(CC_flag == 1){
      CC_flag = 0;
      transmit(0x35, CC_activate, 0xFF, 0xFF, 0xFF);
    }
    else if(CV_flag == 1){
      CV_flag = 0;
      transmit(0x36, CV_activate, 0xFF, 0xFF, 0xFF);
    }
    
    else{
      if(Serial.available() > 0){
        Serial.readBytes(buff, 5);

        if(buff[0] == 0x30){
          //Panel Voltage
          PV_int = buff[1];
          PV = PV_int/10.0;
          dtostrf(PV, 10, 6, buffer);
          client.publish("panel_voltage", buffer);

          //Panel current 
          PI_int = buff[2];
          PC = PI_int/10.0;
          dtostrf(PC, 10, 6, buffer);
          client.publish("panel_current", buffer);

          //Battery voltage
          BV_int = buff[3];
          BV = BV_int/10.0;
          dtostrf(BV, 10, 6, buffer);
          client.publish("battery_voltage", buffer);

          //Battery current
          BI_int = buff[4];
          BI = BI_int/10.0;
          dtostrf(BI, 10, 6, buffer);
          client.publish("battery_current", buffer);

          panel_power = PV*PC;
          battery_power = BV*BI;
          dtostrf(panel_power, 10, 6, buffer);
          client.publish("panel_power", buffer);
          dtostrf(battery_power, 10, 6, buffer);
          client.publish("battery_power", buffer);


        }
        else if(buff[0] == 0x31){
          //Panel Azi 
          PA_int = ((buff[1] << 8) | buff[2]);
          PA = PA_int/100.0;
          dtostrf(PA, 10, 6, buffer);
          client.publish("panel_azi", buffer);

          //Sun Azi
          SA_int = ((buff[3] << 8) | buff[4]);
          SA = SA_int/100.0;
          dtostrf(SA, 10, 6, buffer);
          client.publish("sun_azi", buffer);
          
        }
        else if(buff[0] == 0x32){
          //Sun Vert
          SVert_int = ((buff[1] << 8) | buff[2]);
          SVert = SVert_int/100.0;
          dtostrf(SVert, 10, 6, buffer);
          client.publish("panel_vert", buffer);

          //Panel Vert
          PVert_int = ((buff[3] << 8) | (buff[4]));
          PVert = PVert_int/100.0;
          dtostrf(PVert, 10, 6, buffer);
          client.publish("sun_vert", buffer);
        }
        if(buff[0] == 0x55){
          //Panel Voltage
          PV_int = buff[1];
          PV = PV_int/10.0;
          dtostrf(PV, 10, 6, buffer);
          client.publish("panel_voltage", buffer);

          //Panel current 
          PI_int = buff[2];
          PC = PI_int/10.0;
          dtostrf(PC, 10, 6, buffer);
          client.publish("panel_current", buffer);

          //Battery voltage
          P_Duty = buff[3];
          itoa(P_Duty, buffer, 10);
          client.publish("scan_duty", buffer);

          panel_power = PV*PC;
          dtostrf(panel_power, 10, 6, buffer);
          client.publish("panel_power_scan", buffer);
        }
        else if(buff[0] == 0x99){
          battery_State = buff[1];
          itoa(battery_State, buffer, 10);
          client.publish("battery_state_confirm", buffer);
        }
      }
    }
  }
}

  // while(state == 1){
  //   //client.loop();
  //   transmit(0x10, 0x01, 0xFF, 0xFF, 0xFF);
  //   waiting = 1;

  //   while(waiting == 1){
  //     if(Serial.available() > 0){
  //       Serial.readBytes(buff, 2);
  //       if((buff[1] == 0x01) && (buff[0] == 0x21)){
  //         waiting = 0;
  //         client.publish("state_confirm", "1");
  //         state = 3;
  //       }
  //     }
  //   }
  // }

  // while(state == 3){ //Tracker Mode transition
  //   client.loop();
    
  //   if(home_flag == 1){
  //     home_flag = 0;

  //     transmit(0x11, 0xFF, 0xFF, 0xFF, 0xFF);
  //   }
  //   else if(trigger_flag == 1){
  //     trigger_flag = 0;

  //     transmit(0x12, 0xFF, 0xFF, 0xFF, 0xFF);
  //   }
  //   else if(disturb_flag == 1){
  //     disturb_flag = 0;
  //     transmit(0x13, (byte)(azi_setpoint>>8), (byte)(azi_setpoint), (byte)(vert_setpoint>>8), (byte)(vert_setpoint));
  //   }
  //   else{
  //     if(Serial.available() > 0){
  //       Serial.readBytes(buff, 17);

  //       PV_int = ((buff[1] << 8) | (buff[2]));
  //       PV = PV_int/100.0;
  //       dtostrf(PV, 10, 6, buffer);
  //       client.publish("state_confirm", buffer);

  //       //Panel current 
  //       PI_int = ((buff[3] << 8) | buff[4]);
  //       PC = PI_int/100.0;
  //       dtostrf(PC, 10, 6, buffer);
  //       client.publish("state_confirm", buffer);

  //       //Battery voltage
  //       BV_int = ((buff[5] << 8) | buff[6]);
  //       BV = BV_int/100.0;
  //       dtostrf(BV, 10, 6, buffer);
  //       client.publish("state_confirm", buffer);

  //       //Battery current
  //       BI_int = ((buff[7] << 8) | buff[8]);
  //       BI = BI_int/100.0;
  //       dtostrf(BI, 10, 6, buffer);
  //       client.publish("state_confirm", buffer);

  //       //Sun Azi
  //       SA_int = ((buff[9] << 8) | buff[10]);
  //       SA = SA_int/100.0;
  //       dtostrf(SA, 10, 6, buffer);
  //       client.publish("state_confirm", buffer);

  //       //Panel Azi 
  //       PA_int = ((buff[11] << 8) | buff[12]);
  //       PA = PA_int/100.0;
  //       dtostrf(PA, 10, 6, buffer);
  //       client.publish("state_confirm", buffer);

  //       //Sun Vert
  //       SVert_int = ((buff[13] << 8) | buff[14]);
  //       SVert = SVert_int/100.0;
  //       dtostrf(SVert, 10, 6, buffer);
  //       client.publish("state_confirm", buffer);

  //       //Panel Vert
  //       PVert_int = ((buff[15] << 8) | (buff[16]));
  //       PVert = PVert_int/100.0;
  //       dtostrf(PVert, 10, 6, buffer);
  //       client.publish("state_confirm", buffer);

  //       //Convert and send to dashboard
  //     }
  //   }
  // }