import processing.serial.*;                        //imports serial library
Serial port = new Serial (this, "COM3", 115200);   //opens COM port for communication with name port

void setup () {
  size (300, 300);
}

void draw () {
  String[] lines = loadStrings("gps.txt");         //open file named "gps.txt" and reads its lines and stores as an array
  float la, lo, lat, lon;
  String lad, lod, alt;
  for (int i = 0; i < lines.length; i++)           //loop for going through lines
  {
    println(lines[i]);                             //prints line i
    String[] list = split(lines[i], ',');          //splits line i with commas and put it in a array
    la = float(list[2]);                           //extracts latitude and changes it to float
    lad = list[3];                                 //extracts latitude direction
    lo = float(list[4]);                           //extracts longitude and changes it to float
    lod = list[5];                                 //extracts longitude direction
    alt = list[9];                                 //extracts altitude as a string
    lat = converter(la);                           //converts latitude to degrees by calling converter function
    lon = converter(lo);                           //converts longitude to degrees by calling converter function

    if (lad.equals("S") == true)                   //checks if latitude direction is equal to S
    {
      lat *= -1;                                   //if true multiplies latitude by -1
    }

    if (lod.equals("W") == true)                   //checks if longitude direction is equal to W
    {
      lon *= -1;                                   //if true multiplies longitude by -1
    }

    println(lat + ", " + lon + ", " + alt);        //prints latitude, longitude, altitude
    String latit = String.valueOf(lat);            //changes latitude to string
    String longit = String.valueOf(lon);           //changes longitude to string
    port.write(latit);                             //Serially writes latitude
    port.write(longit);                            //Serially writes longitude
    port.write(alt);                               //Serially writes altitude

  }
  println(" ");
}

float converter(float l)                           //converts DDMM.MMMM coordinates to degrees from minutes
{
    float a = l % 100;
    float b = a / 60;
    float c = (l - a) / 100;
    float d = c + b;
    return d;
}
