#include "tello.hpp"

/*#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"*/
/* Functions:
    [Option 1] Creating a Tello class that in the constructor creates the both sockets and makes de
   command message with the drone. It'd be fine to check how to handle error messages. Since, that
   'd finish the program and destructor 'd be called. IMPORTANT: Close sockets and free ports.

    - connection () en plan abrir la comunicacion con el tello. Simplemente enviar "commmand" y
   esperar a que haya respuesta.
    - getState () filter and divide the state in different variables for make easy the following
   tsks.
*/

int main(int argc, char **argv) {
  (void)argc;
  (void)argv;
  Tello *tello;
  tello = new Tello;

  tello->sendCommand("takeoff");
  sleep(2);
  std::cout << tello->speedMotion(0, 0, 0, 30) << std::endl;
  sleep(2);
  tello->sendCommand("land");
  sleep(5);

  /*bool cent = false;

  for (;;){
      cout<<"IMU info ..."<<endl;
      imu = tello->getIMU();

      for (int i=0; i<imu.size(); i++){
          cout<<imu[i].x<<endl;
          cout<<imu[i].y<<endl;
          cout<<imu[i].z<<endl;
      }
      sleep(1);
  }*/

  return 0;
}

/*
    SocketUdp commandSender("192.168.10.1", 8889, 1024);
    commandSender.bindServer();
    tello = new Tello;
        commandSender.sending(msgs);
        rec = commandSender.receiving();
    }while (rec.length()<1);
    cout<<rec<<endl;
    //////////////////////////////////////////////////////////////
    SocketUdp getState("0.0.0.0", 8890);
    getState.bindServer();
    while (1){
        rec = getState.receiving();
        if (rec.length()>2){
            cout<<rec<<endl;
        }
    }
    //////////////////////////////////////////////////////////////

    do{
        commandSender.sending("streamon");
        cout<<"que porculo"<<endl;
    //rec = video.receiving();

    }while (commandSender.receiving().length()<5);

    VideoCapture img{"udp://0.0.0.0:11111", CAP_FFMPEG};
    while (true){
        Mat frame;
        img >> frame;
        if (!frame.empty())
        {
            imshow("Stream", frame);
        }
        if (waitKey(1) == 27)
        {
            break;
        }
    }*/
