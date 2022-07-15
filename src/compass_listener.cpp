#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf2_msgs/TFMessage.h>
#include <pigpio.h>
#define I2C_ADDR 0x0E
void callback(const geometry_msgs::TransformStamped& t){
    //ROS_INFO_STREAM("I heard " << t.transform.rotation.x);
            
}

int main(int argc, char **argv){
    ros::init(argc, argv, "compass_listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("vicon/wand/wand", 1000, callback);
    
    // init i2ci

    //constants
    const int ID = 0xC4;
    const int STATUS_VAL = 0x00;
    
    gpioInitialise();
    bsc_xfer_t xfer;
    int status;
    xfer.control = (0x0E << 16) | 0x305;
    //memcpy(xfer.txBuf, "A", 1);
    //xfer.txCnt = 1;
    xfer.rxCnt = 0;
    
    while (ros::ok()){
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
        status = bscXfer(&xfer);
        
        if (status >= 0){
            if(xfer.rxCnt > 0){
                ROS_INFO_STREAM("Count: " << xfer.rxCnt << " rxMsg: " << xfer.rxBuf[0]);
                
                switch(xfer.rxBuf[0]){
                    
                    //DATA REGISTER
                    case 0x01:
                        // NEED TO GET DATA
                        break; 

                    //Status register
                    case 0x00:
                        ROS_INFO_STREAM("GOT 0x00");
                        memcpy(xfer.txBuf, &STATUS_VAL, 1);
                        xfer.txCnt = 1;
                        memset(xfer.rxBuf, '\n', sizeof(char)*BSC_FIFO_SIZE);
                        status = bscXfer(&xfer);
                        if(status <= 0){
                            ROS_INFO_STREAM("FAILED TO SEND STATUS\nStatus: " << status);
                        } else {
                            xfer.txCnt = 0;
                        }
                        
                        break;

                    //Respond with ID if ID register is requested
                    case 0x07:
                        ROS_INFO_STREAM("GOT 0x07");
                        memcpy(xfer.txBuf, &ID, 1);
                        xfer.txCnt = 1;
                        memset(xfer.rxBuf, '\n', sizeof(char)*BSC_FIFO_SIZE);
                        status = bscXfer(&xfer);
                        if(status <= 0){
                            ROS_INFO_STREAM("FAILED TO SEND ID\nStatus: " << status);
                        } else {
                            xfer.txCnt = 0;
                        }

                        break;
                    
                        //These are config registers we dont need to respond
                    //and the configs are the same every time
                    case 0x10:
                        //Do nothing
                        break;
                    case 0x11:
                        //Do nothing
                        break;
                    
                    default:
                        ROS_INFO_STREAM("GOT " << xfer.rxBuf[0]);
                }

                /* 
                if(xfer.rxBuf[0] == 0x01){
                    ROS_INFO_STREAM("GOT 0x01");
                } else {
                    ROS_INFO_STREAM("GOT " << xfer.rxBuf[0]);
                }
                */
            }                
        }
        
    }
    //ros::spin();
}
