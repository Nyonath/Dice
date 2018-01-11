%7.12.2016, Michael Meindl, class to act as tcp/ip-client for the BBB
classdef CClient < handle
   properties
       mSocket              %TCP/IP-Socket
       mADCValues           %ADC-Values []
       mX1__dd              %X-Acceleration-Values from Sensor 1 []
       mX2__dd              %X-Acceleration-Values from Sensor 2 []
       mY1__dd              %Y-Acceleration-Values from Sensor 1 []
       mY2__dd              %Y-Acceleration-Values from Sensor 2 []
       mPhi1__d             %Z-Angular-Velocity-Values from Sensor 1 []
       mPhi2__d             %Z-Angular-Velocity-Values from Sensor 2 []
       mTorque              %Torque-Values [Nm]
       mPhi__d              %(d Phi)/(dt) from the Gyros [rad/sec]
       mPhi_A               %Phi from the acceleration values [rad] 
       mPhi_C               %Phi from the complementary values [rad]
       mPhi_G               %Phi from the gyroscope values [rad]
       mPsi__d              %(d Psi)/(dt) from the adc-values [rad/sec]
   end
   
   methods
       function this = CClient()
           this.mSocket = tcpip('192.168.9.1', 40000,...
                                'NetworkRole', 'Client',...
                                'Terminator', '');
           this.mSocket.ByteOrder = 'littleEndian';
           fopen(this.mSocket);
           
           mADCValues = [];
           mX1__dd    = [];
           mX2__dd    = [];
           mY1__dd    = [];
           mY2__dd    = [];
           mPhi1__d   = [];
           mPhi2__d   = [];
           mTorque    = [];
           mPhi__d    = [];
           mPhi_A     = [];
           mPhi_C     = [];
           mPhi_G     = [];
           mPsi__d    = [];
       end
       function readValues(this)
           %Wait for a package to arrive
           containerSize = 2 + 4 + 6 + 6 + 20;
           while(this.mSocket.BytesAvailable < containerSize)
               ; 
           end
           
           this.mX1__dd    = [this.mX1__dd, double(fread(this.mSocket, 1, 'int16'))];
           this.mY1__dd    = [this.mY1__dd, double(fread(this.mSocket, 1, 'int16'))];
           this.mPhi1__d   = [this.mPhi1__d, double(fread(this.mSocket, 1, 'int16'))];
           padding         = double(fread(this.mSocket, 1, 'int16'));
           this.mX2__dd    = [this.mX2__dd, double(fread(this.mSocket, 1, 'int16'))];
           this.mY2__dd    = [this.mY2__dd, double(fread(this.mSocket, 1, 'int16'))];
           this.mPhi2__d   = [this.mPhi2__d, double(fread(this.mSocket, 1, 'int16'))];
           padding         = double(fread(this.mSocket, 1, 'int16'));
           this.mPhi_A     = [this.mPhi_A, double(fread(this.mSocket, 1, 'float32'))];
           this.mPhi_C     = [this.mPhi_C, double(fread(this.mSocket, 1, 'float32'))];
           this.mPhi_G     = [this.mPhi_G, double(fread(this.mSocket, 1, 'float32'))];
           this.mPhi__d    = [this.mPhi__d, double(fread(this.mSocket, 1, 'float32'))];
           this.mPsi__d    = [this.mPsi__d, double(fread(this.mSocket, 1, 'float32'))];
           this.mTorque    = [this.mTorque, double(fread(this.mSocket, 1, 'float32'))];
           this.mADCValues = [this.mADCValues, double(fread(this.mSocket, 1, 'uint16'))]; 
           padding         = double(fread(this.mSocket, 1, 'int16'));
       end
   end
end
