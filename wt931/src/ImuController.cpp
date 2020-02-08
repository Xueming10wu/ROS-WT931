#include "wt931/ImuController.h"

ImuController::ImuController()
{
    //私有参数 获取
    ros::NodeHandle nh_private("~");
    nh_private.param<string>("serialport_name", serialport_name, "/dev/wt931");
    nh_private.param<int>("baudrate", baudrate, 921600);
    nh_private.param<string>("pub_topic_name" ,pub_topic_name);
    

    //串口初始化
    Timeout timeout = Timeout::simpleTimeout(1000);
    try
    {
        serialPort = new Serial(serialport_name, baudrate, timeout);
    }
    catch (exception e)
    {
        cout << "SerialPort Opened failed" << endl;
        exit(0);
    }

    //检查串口是否正常
    if(serialPort->isOpen()){cout << "SerialPort isOpen." << endl;}
    else{cout << "SerialPort Opened failed" << endl;return;}

    //通信协议初始化
    protocol = new CJY901();

    //分配内存
    rxBuffer = new uint8_t [1024];
    for (size_t i = 0; i < 1024; i++)
    {
        rxBuffer[i] = 0;
    }
    

    //休眠3s,丢弃掉串口中错误的信息
    //usleep(3000000);
    //serialPort->flush ();

    //发布者 接收者 初始化
    publisher = nh.advertise<sensor_msgs::Imu>("/imu_data", 1000);

	//ros系统时间
	time_current = ros::Time::now();
    msg.header.frame_id = "base_link";

    //接收是否可以进行正常的读取
    uint8_t canReadCount = 0;
    bool canRead = false;

    for (size_t i = 0; i < 9; i++)
    {
        msg.orientation_covariance[i] = 0;
        msg.angular_velocity_covariance[i] = 0;
        msg.linear_acceleration_covariance[i] = 0;
    }


    while (ros::ok())
    {
        if(serialPort->available() > 44 )
        {   
            serialPort ->read(rxBuffer,44);
            for (size_t i = 0; i < 44; i++)
            {
                
                if (protocol->CopeSerialData(rxBuffer[i]))
                {
                    canReadCount += 1;
                    //cout << int(rxBuffer[i]) << "\n";
                }
                if (canReadCount >= 4)
                {
                    canReadCount = 0;
                    canRead = true;
                }
            }
        }
        
        /*
        else if (serialPort ->available()> 1024)
        {
            serialPort ->flushInput();
        }
        */
        
        else
        {
            usleep(1800);
            //cout << "buffer queue " << serialPort->available() << "\n";
        }
        
        if (canRead)
        {
            canRead = false;
            publishImu();
        }
        ros::spinOnce();
        
    }
}

ImuController::~ImuController()
{
    
}

void ImuController::publishImu()
{
    msg.header.stamp = ros::Time::now();
    msg.orientation.x = float(protocol->stcQuater.q0)/32768;
    msg.orientation.y = float(protocol->stcQuater.q1)/32768;
    msg.orientation.z = float(protocol->stcQuater.q2)/32768;
    msg.orientation.w = float(protocol->stcQuater.q3)/32768;

    
    msg.linear_acceleration.x = float(protocol->stcAcc.a[0])*16*g /32768;
    msg.linear_acceleration.y = float(protocol->stcAcc.a[1])*16*g /32768;
    msg.linear_acceleration.z = float(protocol->stcAcc.a[2])*16*g /32768;


    msg.angular_velocity.x = float( protocol->stcGyro.w[0] ) * 2000 * PI / ( 32768 * 180);
    msg.angular_velocity.y = float( protocol->stcGyro.w[1] ) * 2000 * PI / ( 32768 * 180);
    msg.angular_velocity.z = float( protocol->stcGyro.w[2] ) * 2000 * PI / ( 32768 * 180);

    publisher.publish(msg);
}



