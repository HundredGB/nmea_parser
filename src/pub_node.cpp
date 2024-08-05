#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "nmea_parser/msg/gpsx.hpp"

using namespace std::chrono_literals;

/*
GGA의 자료형을 설명하는 주석
$GPGGA,123313,4704.8062,N,01525.3878,E,1,09,1.1,359.7,M,43.7,M,,*4E
$GPGGA,HHMMSS.ss,AAAA.AAAA,la,OOOO.OOOO,lo,Q,NN,D.D,H.H,h,G.G,g,A.A,RRRR*CS

0	Message ID $GPGGA
1	HHMMSS.ss 	Time of position fix (UTC)
2	AAAA.AAAA 	Latitude degree and minutes (ddmm.mmmmmm)
3	la 	Direction of latitude (North or South)
4	OOOO.OOOO 	Longitude degree and minutes (dddmm.mmmmmm)
5	lo 	Direction of longitude (East or West)
6	Q 	GPS quality indicator:

    	0: no fix available
    	1: GPS fix
    	2: Differential GPS fix (DGPS)
      3: PPS fix
      4: Real Time Kinematic fix (RTK)
      5: Float RTK
    	6: Estimated (dead reckoning)
      7: Manual input mode
      8: simulation mode

7	NN 	Number of satellites in use (00 − 12)
8	D.D 	horizontale dilution of precision (meter)
9	H.H 	Antenna altitude (meter)
10	h 	Unit of antenna altitude (meter)
11	G.G 	geoidal separation the difference between the WGS-84 earth ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid
12	g 	Unit of geoidal separation (meter)
13	A.A 	Age of differential GPS data, time in seconds since last SC104 type 1 or 9 update, null field when DGPS is not used
14	RRRR 	Differential reference station ID (0000 to 1023)
15	CS 	Checksum
*/

/*GGA의 메세지 형태를 정의함 각각 시간,위도,경도,고도,*/
struct messageGGA
{
  std::string UTCtime;
  double latitude;
  double longitude;
  double altitude;
  double dilution;
  double separation;
  int fix;
  int satellites;
};
/*satellite의 메세지 자료형, 서비스에서만 쓰이므로 토픽은 상관X*/
struct satellite
{
  /* data */
  unsigned int id;
  int elevation; // -90 to 90 degrees 
  unsigned int azimuth; // 0 to 360 방위각
  unsigned int SNR; // 0 to 100 in most cases 신호대비 잡음비
  unsigned int type; // 1: GPS, 2:Glonas, 3: Beidou, 4: Galileo
};
/*RMC의 메세지 형태를 정의*/
struct messageRMC
{
  std::string UTCtime;
  double latitude;
  double longitude;
  double speed;
  double degrees;
  std::string date;
  std::string fix;
  int satellites;
};
/*HDT의 메세지 형식 정의. 사실상 HDT는 방위각만 존재*/
struct messageHDT
{
  double azimuth; // 방위각
};


// this must not be part of the class, leads to a compiler error if so
std::vector <struct satellite> sat_monitor_;


class GPSPublisher : public rclcpp::Node
{
  public:
    GPSPublisher(): Node("gps_publisher"), initialized_(false), newdata_GGA(false), newdata_RMC(false), newdata_HDT(false), run_(false)
    {
      // 메세지 구조 초기화
      gga_.UTCtime=std::string();
      gga_.latitude=0.0;
      gga_.longitude=0.0;
      gga_.altitude=0.0;
      gga_.dilution=0.0;
      gga_.separation=0.0;
      gga_.fix=0;
      gga_.satellites=0;

      // 파라미터 정의
      this->declare_parameter<std::string>("comm_port", "/dev/ttyS0");
      rcl_interfaces::msg::IntegerRange range;
      range.from_value = 4800;
      range.step = 1; 
      range.to_value = 115200;
      rcl_interfaces::msg::ParameterDescriptor comm_speed_descriptor;
      comm_speed_descriptor.description = "Serial interface speed setting in Baud";
      comm_speed_descriptor.integer_range.push_back(range);
      this->declare_parameter("comm_speed", rclcpp::ParameterValue(4800), comm_speed_descriptor);
      run();
    }
    ~GPSPublisher()
    {
      gpsConnection_.close();
    }
    
  private:
    void timer_callback();
    void run();
    double safe_stod(std::string& convert);
    double convert_longlat(std::string& convert);
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nmea_parser::msg::Gpsx>::SharedPtr publisher_;
    int openConnection(void);
    int closeConnection(void);
    int readMessage(void);
    int preprocessMessage(std::string* message);
    
    std::fstream gpsConnection_; //파일에 대한 입력 및 출력을 지원하는 클래스.
    struct messageGGA gga_;
    struct messageRMC rmc_;
    struct messageHDT hdt_;
    
    bool initialized_;
    bool newdata_GGA;
    bool newdata_RMC;
    bool newdata_HDT;
    bool run_;

};
 
 /*실질적으로 퍼블리셔를 실행시키는 구문*/
void GPSPublisher::run(void)
{
  publisher_ = this->create_publisher<nmea_parser::msg::Gpsx>("gpsx", 10); //퍼블리셔는 msg 폴더의 Gpsx.msg의 형식에 맞추어 토픽을 전송
  timer_ = this->create_wall_timer(50ms, std::bind(&GPSPublisher::timer_callback, this));
}
 

// 성공하면 0을, 실패하면 음수값을 반환. 각 음수의 값에 따라 어느 부분에 오류가 발생했는지 파악 가능
int GPSPublisher::openConnection(void)
{
  struct termios tty;
  int fd;
  std::string serial_portp;
  int serial_speed;
  
  this->get_parameter("comm_port", serial_portp);
  this->get_parameter("comm_speed", serial_speed);
  RCLCPP_INFO(this->get_logger(), "Opening port: %s speed: %d",serial_portp.c_str(),serial_speed);

  fd = open (serial_portp.c_str(), O_RDWR | O_NOCTTY); //시리얼 연결 명렁어.
  if(fd<0)
    return -2;
    
  // get the current setting, we assume that 8N1 is set TODO: fix so that this is always set
  if(tcgetattr(fd, &tty) != 0)
    RCLCPP_INFO(this->get_logger(), "Error %d from tcgetattr: %s", errno, strerror(errno));
  tty.c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;         /* 8-bit characters */
  tty.c_cflag &= ~PARENB;     /* no parity bit */
  tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
  tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

  // setup for non-canonical mode
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  // fetch bytes as they become available
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 1;
  
  // set baud rate to the parameter comm_speed
  if(serial_speed==4800)
    cfsetispeed(&tty, B4800);
  else if(serial_speed==9600)
    cfsetispeed(&tty, B9600);    
  else if(serial_speed==19200)
    cfsetispeed(&tty, B19200);
  else if(serial_speed==38400)
    cfsetispeed(&tty, B38400);
  else if(serial_speed==57600)
    cfsetispeed(&tty, B57600);
  else if(serial_speed==115200)
    cfsetispeed(&tty, B115200);

  
  if (tcsetattr(fd, TCSANOW, &tty) != 0) // 터미널 설정 오류 검사
    RCLCPP_ERROR(this->get_logger(), "Error %d from tcsetattr: %s",errno,strerror(errno));
  close(fd);

  gpsConnection_.open(serial_portp.c_str(), std::fstream::in | std::fstream::out);
  if(!gpsConnection_.is_open())
  {
    RCLCPP_ERROR(this->get_logger(),"Input stream could not be opened!");
    return -1;    
  }
  
  initialized_=true;
  RCLCPP_INFO(this->get_logger(),"Successfully opened serial connection to GPS");

  return 0;
}

// returns 0 on success <0 on error
int GPSPublisher::closeConnection(void)
{
  if(gpsConnection_.is_open())
  {
    gpsConnection_.close();
  }
  initialized_=false;
  return 0;
}

// string to duoble. nmea메세지는 스트링형태로 수신되므로 double로 바꿔주는 처리.
double GPSPublisher::safe_stod(std::string& convert)
{
  try
  {          
    return(std::stod(convert));
  }
  catch(const std::invalid_argument& ia)
  {
    //RCLCPP_WARN(this->get_logger(),"Could not convert" + convert + " reason: "+ std::string(ia.what()));
    return(NAN); 
  }      
}

// 경도, 위도를 계산하는 구문. string을 double로 변경한 다음 계산하여 반환
double GPSPublisher::convert_longlat(std::string& convert)
{
  double value=safe_stod(convert);
  if(value==NAN)
    return value;
  double dDegrees=floor(value/100); // 반내림 ex) 4.2 -> 4.0. 위도 경도값을 100으로 나눈 뒤 반내림.
  double dMinutes=fmod(value,100); // 소수점 제외 나머지 연산. 위도 경도값을 100으로 나눈 나머지를 계산
  return(dDegrees+dMinutes/60.0); // 위도 경도값을 100으로 나눈 뒤 나머지는 60으로 나누고 몫은 반내림하여 둘을 더한다. 이는 GPS 데이터를 WGS84 형식으로 변환하기 위함이다.
}

// returns 0 on success
// -1: invalid start character
// -2: No asterisk indicating start of checksum
// -3: Checksum does not match
int GPSPublisher::preprocessMessage(std::string* message)
{
  // check checksum 8 Bit XOR for everything between (not including) $ and *
  short int cs_calculated=0;
  short int cs_recieved=0;
  size_t found=0;
  int count=0;

  if((message->at(0)!='$')&&(message->at(0)!='!'))
    return -1;
  found=message->find('*');
  if(found==0)
    return -2;
  cs_recieved=strtol((message->substr(found+1,2)).c_str(),NULL,16);
  for(count=1;count<(int)found;count++)
    cs_calculated ^= message->at(count);
  if(cs_calculated!=cs_recieved)
    return -3;
  // replace the ',,' for empty values with ', ,' for following tokenization
  do
  {
    found=message->find(",,"); // nmea데이터 사이에 null값이 존재하여 ,,로 나타날 경우 사이에 공백을 넣어 토큰화하기 위함.
    if(found!=std::string::npos)
      message->replace(found,2,", ,");
  }
  while(found!=std::string::npos);

  return 0;
}

// returns 0 on success <0 on error
int GPSPublisher::readMessage(void)
{
  if(initialized_)
  {
    std::string msgRead;
    std::vector <std::string> tokens;
    std::string intermediate;
    
    int c = gpsConnection_.peek();  // 그 다음 문자를 확인하여 오류가 있다면 EOF를 반환한다.
    if(c==EOF)
      return -10;
  
    std::getline(gpsConnection_,msgRead); // 데이터를 읽어오는 부분
    if(preprocessMessage(&msgRead)<0) // 읽은 데이터를 preprocessMessage로 보낸 뒤 전처리과정에서 오류가 발생했다는 리턴값이 나타날 경우.
    {
      std::cout << "Check of message read failed: " << msgRead << std::endl;
      return -11;
    }

    // check if proper token
    // $GP: GPS reciever
    // $GN: Combination of different satellite positioning systems
    // $GL: Glonass reciever
    // $BD: Beidou reciever
    // $GA: Galileo reciever
    if(msgRead.compare(0,3,"$GP")!=0&&msgRead.compare(0,3,"$GN")!=0&&msgRead.compare(0,3,"$GL")&&msgRead.compare(0,3,"$BD")&&msgRead.compare(0,3,"$GA"))
    {
      RCLCPP_ERROR(this->get_logger(),"unknown start token of message: %s",msgRead.c_str());
    }
    //RCLCPP_INFO(this->get_logger(), "Read line: "+ msgRead);    
    
    // create stringstream for tokenization
    std::stringstream check1(msgRead); // 읽어온 데이터를 스트링스트림을 통해 수정 가능하게 변환
    // Tokenizing with ',' 
    while(getline(check1, intermediate, ',')) //스트링스트림 된 데이터를 , 기준으로 나누어 토큰화한다. nmea 데이터는 ,를 기준으로 구분되기 떄문
      tokens.push_back(intermediate); 

    // is this one of the implemented messages?
    if(msgRead.compare(3,3,std::string("GGA"))==0) // 문자열 비교. str.compare(a,b,string) --> str의 a부터 b개의 문자열을 string과 비교. 동일하다면 0이 반환된다. 여기서는 4번째 문자열부터 3개의 문자를 본다 $ABCDE 에서 CDE
    {
      newdata_GGA = true;
      //std::cout << "Got new GGA data" << std::endl;
      // this is the GGA message
      for(unsigned int i=1;i<tokens.size();i++)
      {
        switch(i)
        {
          case 0: // will not happen...
        
          break;
          case 1: // this is UTC time
            gga_.UTCtime= tokens[i];
          break;
          case 2: // this is latitude
            gga_.latitude=convert_longlat(tokens[i]);
          break;
          case 3: // orientation North or South
            if((tokens[i].compare("N")!=0) and (tokens[i].compare("n")!=0))
              // southern hemisphere
              gga_.latitude*=-1;
          break;
          case 4: // this is longitude
            gga_.longitude=convert_longlat(tokens[i]);
          break;
          case 5: // orientation East or West
            if((tokens[i].compare("E")!=0) and (tokens[i].compare("e")!=0))
              // western hemisphere
              gga_.longitude*=-1;
          break;
          case 6: // quality of fix, 0 no fix, 1 fix, 2 ground augmented
            gga_.fix=safe_stod(tokens[i]);
          break;
          case 7: // number of sattelites
            gga_.satellites=safe_stod(tokens[i]);
          break;
          case 8: // horizontal dilution of precision
            gga_.dilution=safe_stod(tokens[i]);
          break;
          case 9: // altitude
            gga_.altitude=safe_stod(tokens[i]);
          break;
          case 10: // unit of altitude, usually Meter
            if((tokens[i].compare("M")!=0) and (tokens[i].compare("m")!=0))
              RCLCPP_ERROR(this->get_logger(),"GPS does not report altitude in meter! %s",tokens[i].c_str()); 
          break;
          case 11: // geoidal separation
            gga_.separation=safe_stod(tokens[i]);
          break;
          case 12:
            if((tokens[i].compare("M")!=0) and (tokens[i].compare("m")!=0))
              RCLCPP_ERROR(this->get_logger(),"GPS does not report geoidal separation in meter! %s",tokens[i].c_str());           
          break;
          // ignoring the rest
        }
      }
      //RCLCPP_INFO(this->get_logger(),"UTCtime: "+ UTCtime +" fix:" + std::to_string(fix) + " satellites: " + std::to_string(satellites) + " dilution: " + std::to_string(dilution) + " separation: "+ std::to_string(separation));
      std::cout << "UTCtime: " << gga_.UTCtime << " fix:" << std::to_string(gga_.fix) << " satellites: " << std::to_string(gga_.satellites) << " dilution: " << std::to_string(gga_.dilution) << " separation: " << std::to_string(gga_.separation) << " latitude: " << std::to_string(gga_.latitude) << " longitude: " << std::to_string(gga_.longitude) << std::endl;
    }
    else if(msgRead.compare(3,3,std::string("RMC"))==0) // RMC 데이터일 경우 선언된 변수에 각 토큰값을 저장
    {
      newdata_RMC = true;
      for(unsigned int i=1;i<tokens.size();i++)
      {
        switch(i)
        {
          case 0: // will not happen...
        
          break;
          case 1: // this is UTC time
            rmc_.UTCtime=tokens[i];
          break;
          case 2: // 장치가 신뢰 가능한지의 여부. A이면 신뢰가능, 그렇지 않다면 V로 나타난다
            rmc_.fix=tokens[i];
            if((tokens[i].compare("V")!=0) and (tokens[i].compare("v")!=0))
              std::cout << "it is Active";
          break;
          case 3: // this is latitude
            rmc_.latitude=convert_longlat(tokens[i]);
          break;
          case 4: // orientation North or South
            if((tokens[i].compare("N")!=0) and (tokens[i].compare("n")!=0))
              // southern hemisphere
              rmc_.latitude*=-1;
          break;
          case 5: // this is longitude
            rmc_.longitude=convert_longlat(tokens[i]);
          break;
          case 6: // orientation East or West
            if((tokens[i].compare("E")!=0) and (tokens[i].compare("e")!=0))
              // western hemisphere
              rmc_.longitude*=-1;
          break;
          case 7: // speed over ground
            rmc_.speed=safe_stod(tokens[i]);
          break;
          case 8: // course over ground
            rmc_.degrees=safe_stod(tokens[i]);
          break;
          case 9: // UTC date
            rmc_.date=tokens[i];
          break;
          // ignoring the rest
        }
      }
      std::cout << "UTCdate: " << rmc_.date << "UTCtime: " << rmc_.UTCtime << " fix: " << rmc_.fix << " latitude: " << std::to_string(rmc_.latitude) << " longitude: " << std::to_string(rmc_.longitude) << "Speed: " << std::to_string(rmc_.speed) << "degrees: " << std::to_string(rmc_.degrees) << std::endl;
    }
    else if(msgRead.compare(3,3,std::string("HDT"))==0)
    {
      newdata_HDT = true;
      for(unsigned int i=1;i<tokens.size();i++)
      {
        switch(i)
        {
          case 0: // will not happen...
        
          break;
          case 1: // 방위각 (도 단위)
            hdt_.azimuth= safe_stod(tokens[i]);
          break;
          case 2: // T : 진북을 기준으로 한 방위임을 나타냄
            if((tokens[i].compare("T")!=0) and (tokens[i].compare("t")!=0))
              // southern hemisphere
              hdt_.azimuth*=-1;
          break;
          // ignoring the rest
        }
      }
      std::cout << "azimuth: " << std::to_string(hdt_.azimuth) << std::endl;
    }
    else
    {
      std::cout << "Unknown message: " << msgRead << std::endl;
    }
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Parameterized interface not initialized");
    return -1;
  }
  return 0;
}

void GPSPublisher::timer_callback()
{
  auto message = nmea_parser::msg::Gpsx(); //데이터 형식 맞춤
  if(!initialized_)
  {
    // retry to establish connection in 1 second
    RCLCPP_INFO(this->get_logger(), "Retry to open port on next timer slot second"); 
    if(openConnection()<0)
      return;
  }
  // read and interprete the messages recieved
  readMessage();


  if(newdata_GGA)
  {
    message.longitude=gga_.longitude;
    message.latitude=gga_.latitude;
    message.altitude=gga_.altitude;
    message.satellites=gga_.satellites;
    message.mode_indicator=gga_.fix;
    message.separation=gga_.separation;
    message.dilution=gga_.dilution;
    if(gga_.UTCtime.length()>0)
    {
      //std::cout << "Converting UTC: "<< UTCtime << std::endl;
      message.utc_time=std::strtod(gga_.UTCtime.c_str(),0);
    }
    //RCLCPP_INFO(this->get_logger(), "Publishing: long: '%f' lat: '%f' alt: '%f'", message.longitude, message.latitude, message.altitude);
    publisher_->publish(message);
    std::cout << "GGA message published";
    newdata_GGA=false;
  }
  else if(newdata_RMC)
  {
    message.longitude=rmc_.longitude;
    message.latitude=rmc_.latitude;
    message.ground_speed=rmc_.speed;
    message.true_course=rmc_.degrees;
    message.reliable=rmc_.fix;
    if(rmc_.date.length()>0)
    {
      //std::cout << "Converting UTC: "<< UTCtime << std::endl;
      message.utc_date=std::strtod(rmc_.date.c_str(),0);
    }
    message.satellites=rmc_.satellites;
    if(rmc_.UTCtime.length()>0)
    {
      //std::cout << "Converting UTC: "<< UTCtime << std::endl;
      message.utc_time=std::strtod(rmc_.UTCtime.c_str(),0);
    }
    //RCLCPP_INFO(this->get_logger(), "Publishing: long: '%f' lat: '%f' alt: '%f'", message.longitude, message.latitude, message.altitude);
    publisher_->publish(message);
    std::cout << "RMC message published";
    newdata_RMC=false;
  }
  else if(newdata_HDT)
  {
    message.azimuth=hdt_.azimuth;
    publisher_->publish(message);
    std::cout << "HDT message published";
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GPSPublisher>());
  rclcpp::shutdown();
  return 0;
}
