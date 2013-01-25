/*
  Web  Server AND Web Client! 
 
 A simple web server that shows the value of the analog input pins.
 using an Arduino Wiznet Ethernet shield. 
 AND 
 A simple web client that does a HTTP request to a URL on a server on the LAN, and presents you the results.
 
 Circuit:
 * Ethernet shield attached to pins 10, 11, 12, 13  
 * Analog inputs attached to pins A0 through A5 (optional)
 
 Bits by David A. Mellis,  Tom Igoe and zoomkat. 
 Assembled and made to work properly by Buzz, Jan 2013. ( davidbuzz@gmail.com ) 
 
 */

#include <SPI.h>
#include <Ethernet.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
//byte ip[] = { 192,168,0,253  };

// fill in an available IP address on your network here,
// for manual configuration:
IPAddress ip(192,168,0,253);
IPAddress gateway( 192, 168, 0, 1 ); // internet access via router
IPAddress subnet( 255, 255, 255, 0 ); //subnet mask
IPAddress remoteserver( 192, 168, 0, 163 ); // a simple web page

#define IOPIN 13

// fill in your DNS address here:
//IPAddress myDns(8,8,8,8);

// Initialize the Ethernet server library
// with the IP address and port you want to use 
// (port 80 is default for HTTP):
EthernetServer localserver(80);

// initialize the library instance:
EthernetClient outgoingclient;

String readString; 


//char remoteserver[] = "www.arduino.cc";

unsigned long lastConnectionTime = 0;          // last time you connected to the server, in milliseconds
boolean lastConnected = false;                 // state of the connection last time through the main loop
const unsigned long postingInterval = 6*1000;  // delay between updates, in milliseconds


void setup()
{
  // start serial port:
  Serial.begin(19200);
  // give the ethernet module time to boot up:
  delay(1000);
  
  // start the Ethernet connection using a fixed IP address and DNS server:
  //Ethernet.begin(mac, ip, myDns);
  Ethernet.begin(mac, ip, gateway, subnet);
  
  // start the local server too. 
  localserver.begin();
  
    // print the Ethernet board/shield's IP address:
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());

}

void loop()
{
  //Serial.print(".");
  
   // check for serial input
  if (Serial.available() > 0) 
  {
    byte inChar;
    inChar = Serial.read();
    if(inChar == 'e')
    {
      Serial.println("ack, doing http client request! ");
      sendGET1(); // call sendGET function
    }
  }  
  
  // listen for incoming clients
  EthernetClient incomingclient = localserver.available();
  if (incomingclient) {
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;
    while (incomingclient.connected()) {
      if (incomingclient.available()) {
        char c = incomingclient.read();
        //Serial.print(c); // debug to show client data on Serial console.
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        
        //read char by char HTTP request into 100 byte buffer, toss rest away! 
        if (readString.length() < 100) {
          //store characters to string 
          readString += c; 
        } 
        
        if (c == '\n' && currentLineIsBlank) {
          // send a standard http response header
          incomingclient.println("HTTP/1.1 200 OK");
          incomingclient.println("Content-Type: text/html");
          incomingclient.println();
          
            incomingclient.println("<HTML>");
            incomingclient.println("<HEAD>");
            incomingclient.println("<TITLE>Buzzs simple html page</TITLE>");
            incomingclient.println("</HEAD>");
            incomingclient.println("<BODY>");

         // and output the links for the button on/off stuff: 
          incomingclient.println("<H2>Buzz's Analog data samples:</H2>");
 
 
          // output the value of each analog input pin
          for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
            incomingclient.print("analog input ");
            incomingclient.print(analogChannel);
            incomingclient.print(" is ");
            incomingclient.print(analogRead(analogChannel));
            incomingclient.println("<br />");
          }
          
          // and output the links for the button on/off stuff: 
          incomingclient.println("<H2>Buzz's simple Arduino button:</H2>");
          
            ///////////////////// control arduino pin
          if(readString.indexOf("on") >0)//checks for on
          {
            digitalWrite(IOPIN, HIGH);    // set pin 4 high
            Serial.println("Led On");
          }
          if(readString.indexOf("off") >0)//checks for off
          {
            digitalWrite(IOPIN, LOW);    // set pin 4 low
            Serial.println("Led Off");
          }

          // report to user revised state of pin! 
          if ( digitalRead(IOPIN) == 0 ) { 
            incomingclient.println("<a href=\"/?on\">LED IS OFF. CLICK TO TURN ON</a>"); 
          } else { 
            incomingclient.println("<a href=\"/?off\">LED IS ON, CLICK TO TURN OFF</a>"); 
          }
          //client.println("<IFRAME name=inlineframe style=\"display:none\" >");          
          //client.println("</IFRAME>");

          incomingclient.println("</BODY>");
          incomingclient.println("</HTML>");
          
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } 
        else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    incomingclient.stop();
   //clearing string for next read
   readString="";
  }
  
  
  //  Serial.println("END");
 
}

//////////////////////////
void sendGET2() //client function to send/receie GET request data.
{
  if (outgoingclient.connect(remoteserver,80)) {
    Serial.println("connected");
    outgoingclient.println("GET /ard/arduino.txt HTTP/1.0");
    outgoingclient.println();
  } 
  else {
    Serial.println("connection failed");
    Serial.println();
  }

  while(outgoingclient.connected() && !outgoingclient.available()) delay(1); //waits for data
  while (outgoingclient.connected() || outgoingclient.available()) { //connected or data available
    char c = outgoingclient.read();
    Serial.print(c);
  }

  Serial.println();
  Serial.println("disconnecting.");
  Serial.println("==================");
  Serial.println();
  outgoingclient.stop();

}

  // now we optionally can do an outgoing client...... 
void sendGET1() {
    // if there's incoming data from the net connection.
    // send it out the serial port.  This is for debugging
    // purposes only:
  
    // if there's no net connection, but there was one last time
    // through the loop, then stop the outgoingclient:
    if (!outgoingclient.connected() && lastConnected) {
      Serial.println();
      Serial.println("disconnecting.");
      outgoingclient.stop();
    }
  
    // if you're not connected, and X seconds have passed since
    // your last connection, then connect again and send data:
    if(!outgoingclient.connected() && (millis() - lastConnectionTime > postingInterval)) {
       // Serial.println("pre-connecting!......");

         // if there's a successful connection:
          if (outgoingclient.connect(remoteserver, 80)) {
            //Serial.println("connecting...");
            // send the HTTP PUT request:
            outgoingclient.println("GET /ard/arduino.txt HTTP/1.0");   // or HTTP/1.1
           // outgoingclient.println("Host: www.arduino.cc");
           // outgoingclient.println("User-Agent: arduino-ethernet");
           // outgoingclient.println("Connection: close");
            outgoingclient.println();
        
            // note the time that the connection was made:
            lastConnectionTime = millis();
            
               Serial.println("connected.");
            //   delay(1); // wait for data
            
          } 
          else {
            // if you couldn't make a connection:
            Serial.println("http connection failed");
           // Serial.println("disconnecting unclean.");
            outgoingclient.stop();
          }
    } else { 
      Serial.print("too soon for next http request, please slow them down to wait at least  ");
      Serial.print(postingInterval/1000);
      Serial.println(" secs between them.");
    }
    
    
    while(outgoingclient.connected() && !outgoingclient.available()) delay(1); //waits for data

    
     //  if we were just connectd by the above block we'll have data, check fgor it now! 
    while (outgoingclient.connected() || outgoingclient.available()) { //connected or data available
      char c = outgoingclient.read();
      Serial.print(c);
    }
   // Serial.println("data done.");
   // Serial.println(outgoingclient.connected());
   // Serial.println(outgoingclient.available());
    
    
    // finally  stop the outgoingclient:
    if (! outgoingclient.connected() ) {
      Serial.println();
      Serial.println("disconnected ok.");
      outgoingclient.stop();
    }


    // store the state of the connection for next time through
    // the loop:
    lastConnected = outgoingclient.connected();
    

}



// this method makes a HTTP connection to the server:
void httpRequest() {
 
}
